"""The ROS session behind the Program step: one container running MoveIt on MuJoCo physics (mock
hardware when the export has no MuJoCo model) and rosbridge on the robot's generated workspace."""
import re
import subprocess
import threading
from pathlib import Path

from app.export.urdf_writer import export_package
from app.ros.cli import workspace_from_zip
from app.ros.program_pkg import RUN_TIMEOUT_S, SESSION_CONTAINER

IMAGE = "cad-to-urdf-jazzy"
REPO = Path(__file__).resolve().parents[3]
DOCKER_DIR = REPO / "docker" / "jazzy"
WORKSPACES = Path.home() / ".cad-to-urdf" / "workspaces"
SAMPLES = 10000
PROGRAM_RE = re.compile(r"^programs/[A-Za-z0-9_]+\.py$")
DOCKER_DOWN_MARKERS = ("docker API", "cannot connect to the docker daemon",
                       "is the docker daemon running", "dockerDesktopLinuxEngine")

_current: dict | None = None
LABEL_FORMAT = ('{{index .Config.Labels "cad-to-urdf.robot"}}|'
                '{{index .Config.Labels "cad-to-urdf.src"}}')


class SessionError(Exception):
    pass


class SessionBusy(SessionError):
    pass


_lock = threading.Lock()
_prep: dict | None = None  # host-side startup: robot, stage, done, total, error, stop, launched
_thread: threading.Thread | None = None


def _docker(*args: str, check: bool = True) -> subprocess.CompletedProcess:
    try:
        return subprocess.run(["docker", *args], capture_output=True, text=True, check=check)
    except FileNotFoundError:
        raise SessionError("Docker is not installed or not on PATH.")
    except subprocess.CalledProcessError as exc:
        err = (exc.stderr or exc.stdout or "").strip()
        low = err.lower()
        if any(m.lower() in low for m in DOCKER_DOWN_MARKERS):
            raise SessionError("Start Docker Desktop.")
        raise SessionError(err.splitlines()[-1] if err else f"docker {args[0]} failed")


def current() -> dict | None:
    """The session's robot and workspace; after a backend restart, recovered from the container labels."""
    global _current
    if _current is None:
        r = _docker("inspect", "-f", LABEL_FORMAT, SESSION_CONTAINER, check=False)
        robot, _, src = r.stdout.strip().partition("|") if r.returncode == 0 else ("", "", "")
        if robot:
            _current = {"robot": robot, "src": Path(src)}
    return _current


def start(robot_name: str, links, joints, bodies_by_link) -> dict:
    """Check fast, then export the workspace and (re)start the container in a background thread."""
    global _prep, _thread
    data = export_package(robot_name, links, joints, bodies_by_link)  # ValueError -> caller
    _docker("info")
    with _lock:
        if _thread is not None and _thread.is_alive():
            raise SessionBusy("Startup is still finishing; try again in a moment.")
        _prep = prep = {"robot": robot_name, "stage": "collisions", "done": 0, "total": SAMPLES,
                        "error": None, "stop": False, "launched": False}
        _thread = threading.Thread(target=_prepare, args=(prep, data), daemon=True)
        _thread.start()
    return {"state": "starting", "robot": robot_name, "stage": "collisions", "done": 0, "total": SAMPLES}


def _prepare(prep: dict, data: bytes) -> None:
    """The slow part of start(): workspace, image if missing, container. Progress goes into prep."""
    global _current
    try:
        ws = WORKSPACES / prep["robot"]
        robot = workspace_from_zip(data, ws, samples=SAMPLES,
                                   on_progress=lambda d, t: prep.update(done=d, total=t))
        if _docker("image", "inspect", IMAGE, check=False).returncode != 0:
            prep.update(stage="image", done=0, total=0)
            _docker("build", "-t", IMAGE, str(DOCKER_DIR))
        with _lock:
            if prep["stop"]:
                return
            _docker("rm", "-f", SESSION_CONTAINER, check=False)
            src = ws / "src"
            _docker("run", "-d", "--name", SESSION_CONTAINER, "-p", "127.0.0.1:9090:9090",
                    "--label", f"cad-to-urdf.robot={robot}", "--label", f"cad-to-urdf.src={src}",
                    "-v", f"{ws}:/ws", "-v", f"{DOCKER_DIR / 'session'}:/session:ro",
                    IMAGE, "bash", "/session/start.sh")
            _current = {"robot": robot, "src": src}
            prep.update(stage="build", done=0, total=0, launched=True)
    except Exception as exc:  # any failure must reach the UI, not die with the thread
        prep["error"] = str(exc) or type(exc).__name__


def logs(lines: int = 30) -> str:
    r = _docker("logs", "--tail", str(lines), SESSION_CONTAINER, check=False)
    return (r.stdout or "") + (r.stderr or "")


def status() -> dict:
    prep = _prep
    if prep is not None and not prep["stop"]:
        if prep["error"]:
            return {"state": "failed", "robot": prep["robot"], "stage": prep["stage"], "error": prep["error"]}
        if not prep["launched"]:
            return {"state": "starting", "robot": prep["robot"], "stage": prep["stage"],
                    "done": prep["done"], "total": prep["total"]}
    cur = current()
    robot = cur["robot"] if cur else None
    r = _docker("inspect", "-f", "{{.State.Status}}", SESSION_CONTAINER, check=False)
    if r.returncode != 0 or not cur:
        return {"state": "stopped", "robot": robot}
    if r.stdout.strip() != "running":
        return {"state": "failed", "robot": robot, "log": logs()}
    r = _docker("exec", SESSION_CONTAINER, "cat", "/tmp/stage", check=False)
    stage = r.stdout.strip() or "build"
    if r.returncode != 0:  # a container started before stage files: only /tmp/ready tells
        ready = _docker("exec", SESSION_CONTAINER, "test", "-f", "/tmp/ready", check=False).returncode == 0
        stage = "ready" if ready else "build"
    if stage == "ready":
        r = _docker("exec", SESSION_CONTAINER, "cat", "/tmp/physics", check=False)
        physics = (r.stdout.strip() if r.returncode == 0 else "") or "off: unknown (session started before physics support)"
        return {"state": "ready", "robot": robot, "physics": physics}
    return {"state": "starting", "robot": robot, "stage": stage, "log": logs(8)}


def stop() -> None:
    global _current, _prep
    with _lock:
        if _prep is not None:
            _prep["stop"] = True
        _prep = None
        _docker("rm", "-f", SESSION_CONTAINER, check=False)
        _current = None


def run_program(program: str) -> dict:
    """Run a program again without the agent (no run count)."""
    if not PROGRAM_RE.match(program):
        raise ValueError("Program path must look like programs/task_001.py.")
    cur = current()
    if not cur:
        raise SessionError("Start a session first.")
    robot = cur["robot"]
    cmd = (f"source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && "
           f"cd /ws/src/{robot}_tasks && PYTHONPATH=/ws/src/{robot}_program:$PYTHONPATH python3 -u -m runner {program}")
    r = _docker("exec", SESSION_CONTAINER, "timeout", str(RUN_TIMEOUT_S), "bash", "-c", cmd, check=False)
    out = (r.stdout or "") + (r.stderr or "")
    return {"ok": "RESULT: ok" in out, "output": out}
