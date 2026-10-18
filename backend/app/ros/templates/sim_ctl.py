"""Container side: one MuJoCo scene per run. Boots the sim at session start, then before every run
resets it (same scene) or restarts only the MuJoCo control node (new scene). MoveIt stays up.
Writes /tmp/sim/hardware (mujoco | mock) for the launch file and /tmp/physics for the app."""
import json
import os
import shutil
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

import scene

SIM = Path("/tmp/sim")
WS_SRC = Path("/ws/src")
RESTART_TIMEOUT = 30.0
NODE = "mujoco_ros2_control_node"


class SimError(Exception):
    pass


def _fixture_dir(ws_src: Path, robot: str) -> Path:
    return ws_src / f"{robot}_tasks" / "meshes"


def _with_paths(objs: list[dict], ws_src: Path, robot: str) -> list[dict]:
    """CAD fixtures carry their mesh's full path in meta.json: robot.py loads it into MoveIt."""
    return [{**o, "path": str(_fixture_dir(ws_src, robot) / o["file"])} if o["shape"] == "mesh" else o for o in objs]


def plan(ws_src: Path, robot: str, tool: str, program_source):
    """(scene.xml text, meta) for the workcell plus the program's SCENE; raises SceneError."""
    robot_mjcf = (ws_src / robot / "mjcf" / f"{robot}.xml").read_text()
    wc_path = ws_src / f"{robot}_tasks" / "workcell.yaml"
    workcell = scene.validate(scene.load_workcell(wc_path.read_text() if wc_path.exists() else None), "workcell.yaml")
    program = scene.validate(scene.read_scene(program_source), "SCENE") if program_source else []
    objs = _with_paths(scene.merge(workcell, program), ws_src, robot)
    xml = scene.build_scene_xml(robot_mjcf, objs, tool, str(ws_src / robot / "meshes"), str(_fixture_dir(ws_src, robot)))
    robot_bodies = {b.get("name") for b in ET.fromstring(robot_mjcf).iter("body")} | {"world"}
    scene.check_start(xml, None, robot_bodies)
    return xml, scene.meta(robot_mjcf, objs)


def objects_meta(ws_src: Path, robot: str, program_source) -> dict:
    """Mock hardware (no MuJoCo model): the same objects, validated, for MoveIt only."""
    wc_path = ws_src / f"{robot}_tasks" / "workcell.yaml"
    workcell = scene.validate(scene.load_workcell(wc_path.read_text() if wc_path.exists() else None), "workcell.yaml")
    program = scene.validate(scene.read_scene(program_source), "SCENE") if program_source else []
    objs = _with_paths(scene.merge(workcell, program), ws_src, robot)
    return {"efforts": {}, "masses": {o["name"]: o["mass"] for o in objs if not o["fixed"]},
            "fixed": [o["name"] for o in objs if o["fixed"]], "objects": objs}


def decide(new_xml: str, loaded_path: Path, alive: bool) -> str:
    """reset only a running sim that already has this exact scene; anything else restarts."""
    return "reset" if alive and loaded_path.exists() and loaded_path.read_text() == new_xml else "restart"


def write_atomic(path: Path, text: str) -> None:
    """Readers (scene_sync, robot.py) never see a half-written file."""
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(text)
    os.replace(tmp, path)


def _sim_alive() -> bool:
    try:
        os.killpg(int((SIM / "control.pid").read_text()), 0)
        return True
    except (FileNotFoundError, ProcessLookupError, ValueError, PermissionError):
        return False


def plugins_yaml() -> str:
    return yaml.safe_dump({"/**": {"ros__parameters": {"mujoco_plugins": {
        "free_joint_state_publisher": {"type": "mujoco_ros2_control_plugins/FreeJointStatePublisherPlugin",
                                       "topic": "/sim/free_joint_states", "publish_rate": 20.0},
        "glue": {"type": "cad_to_urdf_glue/GluePlugin"}}}}})


def controller_names(has_gripper: bool, mobile: bool = False) -> list[str]:
    return ["arm_controller"] + (["gripper_controller"] if has_gripper else []) + (["base_controller"] if mobile else [])


def _controllers() -> list[str]:
    robot = next(p.name[:-len("_moveit_config")] for p in WS_SRC.glob("*_moveit_config"))
    chain = __import__(f"{robot}_kinematics").load().chain
    return controller_names(chain.gripper is not None, chain.mobile is not None)


def _robot_and_tool():
    robot = next(p.name[:-len("_moveit_config")] for p in WS_SRC.glob("*_moveit_config"))
    return robot, __import__(f"{robot}_kinematics").load().chain.tip


def _ros(cmd: str, timeout: float) -> subprocess.CompletedProcess:
    return subprocess.run(["bash", "-c", f"source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && {cmd}"],
                          capture_output=True, text=True, timeout=timeout)


def _log_tail(n: int = 20) -> str:
    p = SIM / "control.log"
    return "\n".join(p.read_text(errors="replace").splitlines()[-n:]) if p.exists() else ""


def _stop_sim():
    pid_file = SIM / "control.pid"
    if not pid_file.exists():
        return
    try:
        pgid = int(pid_file.read_text())
        os.killpg(pgid, signal.SIGINT)
        for _ in range(50):
            os.killpg(pgid, 0)
            time.sleep(0.1)
        os.killpg(pgid, signal.SIGKILL)
    except (ProcessLookupError, ValueError):
        pass
    pid_file.unlink(missing_ok=True)


def control_node_cmd(robot: str) -> str:
    """The MuJoCo control node. It reads the robot (with hardware:=mujoco and mujoco_model =
    /tmp/sim/scene.xml) from /robot_description, which MoveIt's robot_state_publisher latches."""
    share = f"/ws/install/{robot}_moveit_config/share/{robot}_moveit_config"
    return (f"ros2 run mujoco_ros2_control ros2_control_node --ros-args "
            f"--params-file {share}/config/ros2_controllers.yaml --params-file {SIM / 'plugins.yaml'} "
            "-r ~/robot_description:=/robot_description")


def _start_sim(robot: str):
    (SIM / "plugins.yaml").write_text(plugins_yaml())
    with open(SIM / "control.log", "w") as log:
        p = subprocess.Popen(
            ["bash", "-c", "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && exec "
             + control_node_cmd(robot)],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    (SIM / "control.pid").write_text(str(p.pid))
    try:
        r = _ros(f"ros2 run controller_manager spawner joint_state_broadcaster {' '.join(_controllers())} "
                 f"--controller-manager-timeout {int(RESTART_TIMEOUT)}", RESTART_TIMEOUT + 10)
        ok = r.returncode == 0
    except subprocess.TimeoutExpired:
        ok = False
    if not ok or p.poll() is not None:
        raise SimError(f"The simulation did not start within {RESTART_TIMEOUT:.0f} s:\n{_log_tail()}")
    _wait_for_arm_controller()


def _wait_for_arm_controller():
    """MoveIt's action client reconnects to a restarted controller on its own, but not instantly:
    a motion sent too early fails with 'Action client not connected'."""
    import rclpy
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("cad_to_urdf_sim_wait")
    try:
        if not ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory").wait_for_server(10):
            raise SimError(f"The arm controller did not come up after the simulation restarted:\n{_log_tail()}")
    finally:
        node.destroy_node()
    time.sleep(1.0)  # ponytail: margin for MoveIt's own client; poll MoveIt's connection if this proves flaky


def _call(node, srv_type, name: str, req, what: str):
    import rclpy
    cli = node.create_client(srv_type, name)
    if not cli.wait_for_service(timeout_sec=10):
        raise SimError(f"The simulation is not answering (no {name}):\n{_log_tail()}")
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
    if not fut.done():
        raise SimError(f"Could not {what}: no answer from {name}")
    return fut.result()


def _reset_sim():
    """reset_world through rclpy (the ros2 CLI costs ~4 s per call). The arm controller is off
    meanwhile: it would otherwise keep commanding the previous run's last pose and swing the arm back."""
    import rclpy
    from controller_manager_msgs.srv import SwitchController
    from mujoco_ros2_control_msgs.srv import ResetWorld
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("cad_to_urdf_sim_ctl")
    try:
        def switch(**kw):
            r = _call(node, SwitchController, "/controller_manager/switch_controller",
                      SwitchController.Request(strictness=SwitchController.Request.STRICT, **kw), "switch controllers")
            if not r.ok:
                raise SimError(f"Could not switch the arm controller ({kw})")
        switch(deactivate_controllers=_controllers())
        r = _call(node, ResetWorld, f"/{NODE}/reset_world", ResetWorld.Request(), "reset the simulation")
        if not r.success:
            raise SimError(f"Could not reset the simulation: {r.message}")
        switch(activate_controllers=_controllers())
    finally:
        node.destroy_node()


def _wait_for_object_poses(names: set[str], timeout: float = 10.0) -> bool:
    """True once MuJoCo's pose plugin has reported every one of `names` on /sim/free_joint_states."""
    import rclpy
    from mujoco_ros2_control_msgs.msg import FreeJointStateArray
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("cad_to_urdf_sim_poses_wait")
    seen: set[str] = set()
    node.create_subscription(FreeJointStateArray, "/sim/free_joint_states",
                             lambda m: seen.update(f.name for f in m.free_joints), 10)
    end = time.time() + timeout
    try:
        while time.time() < end and not names <= seen:
            rclpy.spin_once(node, timeout_sec=0.1)
        return names <= seen
    finally:
        node.destroy_node()


def _start_with_object_poses(robot: str, movable: set[str]) -> None:
    """Start the sim and make sure it reports where its loose objects are. After a restart the pose plugin
    sometimes comes up silent (container check: Publisher count 0 on /sim/free_joint_states while the arm
    moved), so robot.py can't see the objects and a grip closes on nothing. Restart once; keep the silent
    start's log (/tmp/sim/silent_start_<n>.log) for diagnosis."""
    for attempt in (1, 2):
        _start_sim(robot)
        if not movable or _wait_for_object_poses(movable):
            return
        if (SIM / "control.log").exists():
            shutil.copy(SIM / "control.log", SIM / f"silent_start_{attempt}.log")
        _stop_sim()
    raise SimError("The simulation started but did not report where the objects are (tried twice). "
                   "Restart the session; the sim logs are in /tmp/sim/silent_start_*.log")


def prepare(program):
    """Before a run: the program's scene (program = path relative to <robot>_tasks/, or None).
    Under mock hardware only the object list is written (robot.py adds it to MoveIt)."""
    SIM.mkdir(parents=True, exist_ok=True)
    robot, tool = _robot_and_tool()
    source = (WS_SRC / f"{robot}_tasks" / program).read_text() if program else None
    hw = SIM / "hardware"
    if not (hw.exists() and hw.read_text().strip() == "mujoco"):
        write_atomic(SIM / "meta.json", json.dumps(objects_meta(WS_SRC, robot, source)))
        return
    xml, info = plan(WS_SRC, robot, tool, source)
    write_atomic(SIM / "meta.json", json.dumps(info))
    try:
        if decide(xml, SIM / "scene.xml", _sim_alive()) == "reset":
            try:
                _reset_sim()
                Path("/tmp/physics").write_text("mujoco")
                return
            except SimError:
                pass  # a stuck sim or controller: start fresh below
        _stop_sim()
        write_atomic(SIM / "scene.xml", xml)  # the sim node loads it at start
        _start_with_object_poses(robot, set(info["masses"]))
        Path("/tmp/physics").write_text("mujoco")
    except SimError as exc:
        _stop_sim()
        (SIM / "scene.xml").unlink(missing_ok=True)  # a failed start never counts as "this scene is loaded"
        Path("/tmp/physics").write_text(f"off: {str(exc).splitlines()[0]}")
        raise


def hardware():
    """Before MoveIt starts: mujoco when the export has a MuJoCo model, else mock (as before)."""
    SIM.mkdir(parents=True, exist_ok=True)
    robot, _ = _robot_and_tool()
    if (WS_SRC / robot / "mjcf" / f"{robot}.xml").exists():
        (SIM / "hardware").write_text("mujoco")
    else:
        (SIM / "hardware").write_text("mock")
        Path("/tmp/physics").write_text("off: the export has no MuJoCo model (see the Physics card in Simulate)")


def boot():
    """After MoveIt is up (it publishes /robot_description): start the sim with the workcell."""
    if (SIM / "hardware").read_text().strip() != "mujoco":
        return
    try:
        prepare(None)  # writes /tmp/physics; every later run's prepare updates it again
    except (scene.SceneError, SimError) as exc:  # runs fail with the same reason until it is fixed
        Path("/tmp/physics").write_text(f"off: {str(exc).splitlines()[0]}")


if __name__ == "__main__":
    {"hardware": hardware, "boot": boot}[sys.argv[1]]()
