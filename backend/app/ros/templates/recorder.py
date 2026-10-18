"""Run replay: records what the 3D view shows during a run (the robot's /tf, MuJoCo's object poses, MoveIt's
scene), so the run can be watched again after it ends. runner.py starts it after the reset and ends it
after the program, failed or not. Recording never fails a run: it prints 'TRACE: none (why)' instead."""
import json
import threading
import time
from pathlib import Path

RATE_S = 0.05          # a frame (every moving link's pose, every object's) 20 times a second
SCENE_S = 0.2          # MoveIt's scene polled 5 times a second, kept only when it changed
MAX_S = 300.0          # never more than 5 minutes (a run times out well before)
SCENE_COMPONENTS = 20  # world object geometry + attached objects: what the live view asks for


def trace_name(program: str, now: float) -> str:
    """runs/<program stem>-<UTC time>.json, next to programs/ in <robot>_tasks."""
    return f"runs/{Path(program).stem}-{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime(now))}.json"


def _structure(scene: dict):
    """What replay draws from a scene, without poses: each world object's id and shape, and which objects are
    held by which link. A scene that only moved isn't kept again (MuJoCo's poses show motion at 20 Hz, and
    the rest of MoveIt's answer, e.g. the joint state, changes all the time)."""
    def shape(o):
        return ([list(p.get("dimensions", [])) for p in o.get("primitives", [])],
                [len(m.get("triangles", [])) for m in o.get("meshes", [])])
    world = (scene.get("world") or {}).get("collision_objects", [])
    held = (scene.get("robot_state") or {}).get("attached_collision_objects", [])
    return {"world": sorted([o.get("id"), shape(o)] for o in world),
            "held": sorted([a.get("link_name"), (a.get("object") or {}).get("id"), shape(a.get("object") or {})] for a in held)}


def _row(t) -> list:
    tr, q = t.transform.translation, t.transform.rotation
    return [t.child_frame_id, t.header.frame_id, tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w]


class Trace:
    """The recording, pure Python (tested on the host): frames, scene changes, the static transforms."""

    def __init__(self, robot: str, program: str, base: str, start: float):
        self.robot, self.program, self.base, self.start = robot, program, base, start
        self.latest: dict[str, list] = {}
        self.static: list[list] = []
        self.poses: dict[str, list] = {}   # MuJoCo's free objects: name -> [x, y, z, qx, qy, qz, qw]
        self.frames: list[dict] = []
        self.scenes: list[dict] = []
        self._last_scene: str | None = None
        self.lock = threading.Lock()

    def tf(self, transforms, static: bool = False) -> None:
        with self.lock:
            for t in transforms:
                (self.static.append if static else lambda r: self.latest.__setitem__(r[0], r))(_row(t))

    def sample(self, now: float) -> None:
        t = now - self.start
        if t > MAX_S:
            return
        with self.lock:
            self.frames.append({"t": round(t, 3), "tf": list(self.latest.values()), "poses": dict(self.poses)})

    def scene(self, now: float, scene: dict) -> None:
        key = json.dumps(_structure(scene), sort_keys=True, default=str)
        if key == self._last_scene or now - self.start > MAX_S:
            return
        self._last_scene = key
        with self.lock:
            self.scenes.append({"t": round(now - self.start, 3), "scene": scene})

    def write(self, path: Path, result: str) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with self.lock:
            data = {"version": 1, "robot": self.robot, "program": self.program, "base": self.base,
                    "static": self.static, "frames": self.frames, "scenes": self.scenes, "result": result}
        tmp = path.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(data, default=str), encoding="utf-8")
        tmp.replace(path)


class _Off:
    """No recording (it could not start): the run carries on and says why."""

    def __init__(self, why: str):
        self.why = why

    def end(self, result: str) -> str:
        return f"TRACE: none ({self.why})"


class Recorder:
    def __init__(self, name: str, trace: Trace, node, executor, thread):
        self.name, self.trace, self.node, self.executor, self.thread = name, trace, node, executor, thread

    def end(self, result: str) -> str:
        try:
            self.executor.shutdown(timeout_sec=2.0)
            self.thread.join(timeout=2.0)
            self.node.destroy_node()
            self.trace.write(Path.cwd() / self.name, result)
            return f"TRACE: {self.name}"
        except Exception as exc:  # the run's result matters more than its recording
            return f"TRACE: none (could not save the recording: {exc})"


def begin(program: str, robot) -> "Recorder | _Off":
    """Start recording the run of `program` (relative to <robot>_tasks, the working directory). robot: the
    connected Robot (rclpy is up); its chain gives the base frame. Never raises."""
    try:
        import rclpy
        from moveit_msgs.msg import PlanningSceneComponents
        from moveit_msgs.srv import GetPlanningScene
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from rosidl_runtime_py.convert import message_to_ordereddict
        from tf2_msgs.msg import TFMessage
        import robot as robot_lib

        trace = Trace(robot_lib.ROBOT, program, robot.chain.base, time.monotonic())
        node = rclpy.create_node("cad_to_urdf_recorder")
        node.create_subscription(TFMessage, "/tf", lambda m: trace.tf(m.transforms), 50)
        node.create_subscription(TFMessage, "/tf_static", lambda m: trace.tf(m.transforms, static=True),
                                 QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        if getattr(robot_lib, "_PHYSICS", False):
            from mujoco_ros2_control_msgs.msg import FreeJointStateArray

            def on_poses(m):
                for s in m.free_joints:
                    p, q = s.pose.pose.position, s.pose.pose.orientation
                    trace.poses[s.name] = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            node.create_subscription(FreeJointStateArray, "/sim/free_joint_states", on_poses, 10)
        cli = node.create_client(GetPlanningScene, "/get_planning_scene")
        pending = []

        def poll():
            if pending or not cli.service_is_ready():
                return
            req = GetPlanningScene.Request()
            req.components.components = SCENE_COMPONENTS
            fut = cli.call_async(req)
            pending.append(fut)

            def done(f):
                pending.clear()
                if f.result() is not None:
                    trace.scene(time.monotonic(), message_to_ordereddict(f.result().scene))
            fut.add_done_callback(done)
        node.create_timer(RATE_S, lambda: trace.sample(time.monotonic()))
        node.create_timer(SCENE_S, poll)
        assert PlanningSceneComponents.WORLD_OBJECT_GEOMETRY | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS == SCENE_COMPONENTS
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        return Recorder(trace_name(program, time.time()), trace, node, executor, thread)
    except Exception as exc:
        return _Off(f"recording could not start: {type(exc).__name__}: {exc}")
