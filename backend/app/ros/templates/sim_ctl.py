"""Container side: one MuJoCo scene per run. Boots the sim at session start, then before every run
resets it (same scene) or restarts only the MuJoCo control node (new scene). MoveIt stays up.
Writes /tmp/sim/hardware (mujoco | mock) for the launch file and /tmp/physics for the app."""
import json
import os
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


def plan(ws_src: Path, robot: str, tool: str, program_source):
    """(scene.xml text, meta) for the workcell plus the program's SCENE; raises SceneError."""
    robot_mjcf = (ws_src / robot / "mjcf" / f"{robot}.xml").read_text()
    wc_path = ws_src / f"{robot}_tasks" / "workcell.yaml"
    workcell = scene.validate(scene.load_workcell(wc_path.read_text() if wc_path.exists() else None), "workcell.yaml")
    program = scene.validate(scene.read_scene(program_source), "SCENE") if program_source else []
    objs = scene.merge(workcell, program)
    xml = scene.build_scene_xml(robot_mjcf, objs, tool, str(ws_src / robot / "meshes"))
    robot_bodies = {b.get("name") for b in ET.fromstring(robot_mjcf).iter("body")} | {"world"}
    scene.check_start(xml, None, robot_bodies)
    return xml, scene.meta(robot_mjcf, objs)


def decide(new_xml: str, loaded_path: Path) -> str:
    return "reset" if loaded_path.exists() and loaded_path.read_text() == new_xml else "restart"


def plugins_yaml() -> str:
    return yaml.safe_dump({"/**": {"ros__parameters": {"mujoco_plugins": {
        "free_joint_state_publisher": {"type": "mujoco_ros2_control_plugins/FreeJointStatePublisherPlugin",
                                       "topic": "/sim/free_joint_states", "publish_rate": 20.0},
        "glue": {"type": "cad_to_urdf_glue/GluePlugin"}}}}})


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


def _start_sim(robot: str):
    share = f"/ws/install/{robot}_moveit_config/share/{robot}_moveit_config"
    desc = _ros(f"xacro {share}/config/{robot}.urdf.xacro hardware:=mujoco mujoco_model:={SIM / 'scene.xml'}", 30)
    if desc.returncode != 0:
        raise SimError(f"Could not build the robot description: {desc.stderr.strip()[-300:]}")
    (SIM / "description.yaml").write_text(yaml.safe_dump({"/**": {"ros__parameters": {"robot_description": desc.stdout}}}))
    (SIM / "plugins.yaml").write_text(plugins_yaml())
    with open(SIM / "control.log", "w") as log:
        p = subprocess.Popen(
            ["bash", "-c", "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && exec ros2 run "
             f"mujoco_ros2_control ros2_control_node --ros-args --params-file {share}/config/ros2_controllers.yaml "
             f"--params-file {SIM / 'description.yaml'} --params-file {SIM / 'plugins.yaml'}"],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    (SIM / "control.pid").write_text(str(p.pid))
    try:
        r = _ros("ros2 run controller_manager spawner joint_state_broadcaster arm_controller "
                 f"--controller-manager-timeout {int(RESTART_TIMEOUT)}", RESTART_TIMEOUT + 10)
        ok = r.returncode == 0
    except subprocess.TimeoutExpired:
        ok = False
    if not ok or p.poll() is not None:
        raise SimError(f"The simulation did not start within {RESTART_TIMEOUT:.0f} s:\n{_log_tail()}")


def _reset_sim():
    """reset_world through rclpy (the ros2 CLI costs ~4 s per call); controllers stay active."""
    import rclpy
    from mujoco_ros2_control_msgs.srv import ResetWorld
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("cad_to_urdf_sim_ctl")
    try:
        cli = node.create_client(ResetWorld, f"/{NODE}/reset_world")
        if not cli.wait_for_service(timeout_sec=10):
            raise SimError(f"The simulation is not answering (no /{NODE}/reset_world):\n{_log_tail()}")
        fut = cli.call_async(ResetWorld.Request())
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
        if not fut.done() or not fut.result().success:
            raise SimError("Could not reset the simulation" + (f": {fut.result().message}" if fut.done() else ""))
    finally:
        node.destroy_node()


def prepare(program):
    """Before a run: the program's scene (program = path relative to <robot>_tasks/, or None)."""
    robot, tool = _robot_and_tool()
    source = (WS_SRC / f"{robot}_tasks" / program).read_text() if program else None
    xml, info = plan(WS_SRC, robot, tool, source)
    (SIM / "meta.json").write_text(json.dumps(info))
    if decide(xml, SIM / "scene.xml") == "reset" and (SIM / "control.pid").exists():
        _reset_sim()
        return
    (SIM / "scene.xml").write_text(xml)
    _stop_sim()
    _start_sim(robot)


def boot():
    SIM.mkdir(parents=True, exist_ok=True)
    robot, _ = _robot_and_tool()
    if not (WS_SRC / robot / "mjcf" / f"{robot}.xml").exists():
        (SIM / "hardware").write_text("mock")
        Path("/tmp/physics").write_text("off: the export has no MuJoCo model (see the Physics card in Simulate)")
        return
    (SIM / "hardware").write_text("mujoco")
    try:
        prepare(None)
        Path("/tmp/physics").write_text("mujoco")
    except (scene.SceneError, SimError) as exc:
        _stop_sim()
        (SIM / "hardware").write_text("mock")
        Path("/tmp/physics").write_text(f"off: {str(exc).splitlines()[0]}")


if __name__ == "__main__":
    {"boot": boot}[sys.argv[1]]()
