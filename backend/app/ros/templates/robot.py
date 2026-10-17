"""Task library for __ROBOT__: the functions LLM-written programs call. MoveIt plans every motion.
Units: metres and radians, in the robot's base frame."""
import importlib
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject, Constraints, JointConstraint,
                             MoveItErrorCodes, OrientationConstraint, PlanningScene,
                             PlanningSceneComponents, PositionConstraint)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPlanningScene
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String

import grip_logic
import sim_errors

ROBOT = "__ROBOT__"
GROUP = "arm"
ATTACH_REACH = 0.02   # m: attach() needs the tool frame this close to the object's surface
STILL = 0.01          # rad/s (m/s): under physics, the next motion waits until every joint is this slow
_kin = importlib.import_module(f"{ROBOT}_kinematics")
_home = importlib.import_module(f"{ROBOT}_kinematics._chain").home
_ERRORS = {v: k for k, v in vars(MoveItErrorCodes).items() if k.isupper() and isinstance(v, int)}
_UNREACHABLE = {"NO_IK_SOLUTION", "INVALID_GOAL_CONSTRAINTS", "GOAL_CONSTRAINTS_VIOLATED"}
_NO_PATH = {"FAILURE", "PLANNING_FAILED", "TIMED_OUT"}
_COLLISION = {"GOAL_IN_COLLISION", "START_STATE_IN_COLLISION", "INVALID_MOTION_PLAN",
              "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE"}
_SIM = Path("/tmp/sim")
_PHYSICS = (_SIM / "hardware").exists() and (_SIM / "hardware").read_text().strip() == "mujoco"


def _meta() -> dict:
    """Written by sim_ctl before the run: motor limits, object masses, fixed names, every object."""
    p = _SIM / "meta.json"
    return json.loads(p.read_text()) if p.exists() else {"efforts": {}, "masses": {}, "fixed": [], "objects": []}


class RobotError(Exception):
    """A plain-language problem the program can fix."""


class Unreachable(RobotError):
    pass


class Collision(RobotError):
    pass


class PlanningFailed(RobotError):
    pass


class UnknownObject(RobotError):
    pass


class GripError(RobotError):
    """The gripper caught nothing, an object doesn't fit, or a held object slipped."""


class MotionError(RobotError):
    """The simulated arm could not do what was planned (weak motor, heavy load, contact)."""


def _matrix(xyz, rpy) -> np.ndarray:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    T = np.eye(4)
    T[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                 [-sp, cp * sr, cp * cr]]
    T[:3, 3] = xyz
    return T


def _rpy(R) -> tuple:
    p = math.asin(-max(-1.0, min(1.0, R[2, 0])))
    if abs(math.cos(p)) < 1e-9:
        return (math.atan2(-R[1, 2], R[1, 1]), p, 0.0)
    return (math.atan2(R[2, 1], R[2, 2]), p, math.atan2(R[1, 0], R[0, 0]))


def _quat(R) -> tuple:
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        k = 2 * math.sqrt(t + 1)
        return (R[2, 1] - R[1, 2]) / k, (R[0, 2] - R[2, 0]) / k, (R[1, 0] - R[0, 1]) / k, k / 4
    if R[0, 0] >= R[1, 1] and R[0, 0] >= R[2, 2]:
        k = 2 * math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2])
        return k / 4, (R[0, 1] + R[1, 0]) / k, (R[0, 2] + R[2, 0]) / k, (R[2, 1] - R[1, 2]) / k
    if R[1, 1] >= R[2, 2]:
        k = 2 * math.sqrt(1 - R[0, 0] + R[1, 1] - R[2, 2])
        return (R[0, 1] + R[1, 0]) / k, k / 4, (R[1, 2] + R[2, 1]) / k, (R[0, 2] - R[2, 0]) / k
    k = 2 * math.sqrt(1 - R[0, 0] - R[1, 1] + R[2, 2])
    return (R[0, 2] + R[2, 0]) / k, (R[1, 2] + R[2, 1]) / k, k / 4, (R[1, 0] - R[0, 1]) / k


def _pose(xyz, rpy) -> Pose:
    T = _matrix(xyz, rpy)
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in xyz)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = _quat(T[:3, :3])
    return p


def _pose_matrix(p: Pose) -> np.ndarray:
    x, y, z, w = p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
    M = np.eye(4)
    M[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    M[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return M


def _fmt(xyz) -> str:
    return "(" + ", ".join(f"{v:.3f}" for v in xyz) + ")"


_NODE = None


def _node():
    global _NODE
    if _NODE is None:
        if not rclpy.ok():
            rclpy.init()
        _NODE = rclpy.create_node("cad_to_urdf_task")
    return _NODE


def _wait(future, timeout: float):
    end = time.time() + timeout
    while not future.done() and time.time() < end:
        rclpy.spin_once(_node(), timeout_sec=0.05)
    if not future.done():
        raise PlanningFailed("MoveIt did not answer in time")
    return future.result()


class Robot:
    def __init__(self):
        n = _node()
        self._kin = _kin.load()
        self.chain = self._kin.chain
        self.joint_names = [j.name for j in self.chain.joints]
        self._move = ActionClient(n, MoveGroup, "/move_action")
        self._exec = ActionClient(n, ExecuteTrajectory, "/execute_trajectory")
        self._cartesian = n.create_client(GetCartesianPath, "/compute_cartesian_path")
        self._apply = n.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self._get = n.create_client(GetPlanningScene, "/get_planning_scene")
        self._joints = {}
        self._velocities = {}
        self._stamp = 0
        n.create_subscription(JointState, "/joint_states",
                              self._on_joints, 10)
        self._glued = set()
        self._glue = n.create_publisher(String, "/sim/glue", 10)
        n.create_subscription(String, "/sim/glued", self._on_glued,
                              QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._holding = None          # (name, object pose in the tool frame when gripped)
        self._sim_poses = {}          # object -> 4x4 world pose, straight from MuJoCo
        if self.chain.gripper is not None:
            from control_msgs.action import ParallelGripperCommand
            self._gripper = ActionClient(n, ParallelGripperCommand, "/gripper_controller/gripper_cmd")
            if not self._gripper.wait_for_server(timeout_sec=30):
                raise PlanningFailed("The gripper controller is not running")
        if _PHYSICS:
            from mujoco_ros2_control_msgs.msg import FreeJointStateArray
            n.create_subscription(FreeJointStateArray, "/sim/free_joint_states", self._on_sim_poses, 10)
        for c in (self._cartesian, self._apply, self._get):
            if not c.wait_for_service(timeout_sec=30):
                raise PlanningFailed(f"MoveIt service {c.srv_name} is not available")
        if not (self._move.wait_for_server(timeout_sec=30) and self._exec.wait_for_server(timeout_sec=30)):
            raise PlanningFailed("MoveIt is not running")

    # --- scene -------------------------------------------------------------------------------
    def _diff(self, world=(), attached=()):
        s = PlanningScene()
        s.is_diff = True
        s.robot_state.is_diff = True
        s.world.collision_objects = list(world)
        s.robot_state.attached_collision_objects = list(attached)
        if not _wait(self._apply.call_async(ApplyPlanningScene.Request(scene=s)), 10).success:
            raise PlanningFailed("MoveIt rejected the scene change")

    def _scene(self) -> tuple[list[str], list[str]]:
        req = GetPlanningScene.Request()
        req.components.components = (PlanningSceneComponents.WORLD_OBJECT_NAMES
                                     | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        sc = _wait(self._get.call_async(req), 10).scene
        return ([o.id for o in sc.world.collision_objects],
                [a.object.id for a in sc.robot_state.attached_collision_objects])

    def _add(self, name, shape, dims, xyz, rpy):
        co = CollisionObject()
        co.id = name
        co.header.frame_id = self.chain.base
        co.pose.orientation.w = 1.0
        co.primitives = [SolidPrimitive(type=shape, dimensions=[float(d) for d in dims])]
        co.primitive_poses = [_pose(xyz, rpy)]
        co.operation = CollisionObject.ADD
        self._diff(world=[co])

    @staticmethod
    def _no_mid_run_objects(name):
        if _PHYSICS:
            msg = f"Objects can't appear mid-run. Declare '{name}' in SCENE at the top of the program (or in workcell.yaml)."
            raise RobotError(msg)

    def add_box(self, name, size_xyz, xyz, rpy=(0, 0, 0)):
        self._no_mid_run_objects(name)
        self._add(name, SolidPrimitive.BOX, size_xyz, xyz, rpy)

    def add_cylinder(self, name, radius, height, xyz, rpy=(0, 0, 0)):
        self._no_mid_run_objects(name)
        self._add(name, SolidPrimitive.CYLINDER, (height, radius), xyz, rpy)

    def _on_glued(self, m):
        self._glued = set(filter(None, m.data.split(",")))

    def _glue_cmd(self, op, name):
        """attach/detach in MuJoCo (cad_to_urdf_glue), waiting until the sim confirms it."""
        done = (lambda: name in self._glued) if op == "attach" else (lambda: name not in self._glued)
        self._glue.publish(String(data=f"{op} {name}"))
        end = time.time() + 2.0
        while not done() and time.time() < end:
            rclpy.spin_once(_node(), timeout_sec=0.05)
        if not done():
            raise MotionError(f"The simulation did not confirm {op} of '{name}'")

    def _attached(self, name, op) -> AttachedCollisionObject:
        a = AttachedCollisionObject()
        a.link_name = self.chain.tip
        a.object.id = name
        a.object.operation = op
        a.touch_links = [self.chain.tip, self.chain.joints[-1].parent]   # flange and the wrist it sits on
        g = self.chain.gripper
        if g is not None:   # the gripper's base and fingers hold it
            a.touch_links += [g.base] + [l for l, (seg, _) in self.chain.links.items()
                                         if l not in (g.base, self.chain.tip) and seg == len(self.chain.joints)]
        return a

    def _gap(self, name) -> float:
        """Distance from the tool frame to the surface of world object `name` (0 inside it)."""
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        co = next(o for o in _wait(self._get.call_async(req), 10).scene.world.collision_objects if o.id == name)
        prim, pp = co.primitives[0], co.primitive_poses[0]
        P = _pose_matrix(co.pose) @ _pose_matrix(pp)
        local = np.linalg.inv(P) @ np.append(self._kin.fk(self._current())[:3, 3], 1.0)
        if prim.type == SolidPrimitive.BOX:
            out = np.maximum(np.abs(local[:3]) - np.asarray(prim.dimensions[:3]) / 2, 0)
        else:                                                          # cylinder: (height, radius)
            h, rad = prim.dimensions[0], prim.dimensions[1]
            out = np.array([max(math.hypot(local[0], local[1]) - rad, 0), max(abs(local[2]) - h / 2, 0)])
        return float(np.linalg.norm(out))

    def attach(self, name):
        if self.chain.gripper is not None:
            return self.grip(name)
        world, attached = self._scene()
        if name not in world:
            raise UnknownObject(f"'{name}' is not a free object in the scene "
                                f"(objects: {', '.join(world + attached) or 'none'})")
        gap = self._gap(name)
        if gap > ATTACH_REACH:
            raise UnknownObject(f"'{name}' is {gap:.2f} m from the tool; move the tool to touch it first "
                                f"(within {ATTACH_REACH * 100:.0f} cm, e.g. 5 mm above its top face)")
        if _PHYSICS:
            if name in _meta()["fixed"]:
                raise UnknownObject(f"'{name}' is fixed (bolted down in the workcell) and can't be picked up")
            self._glue_cmd("attach", name)
        self._diff(attached=[self._attached(name, CollisionObject.ADD)])

    def detach(self, name):
        if self.chain.gripper is not None:
            return self.release()
        _, attached = self._scene()
        if name not in attached:
            raise UnknownObject(f"'{name}' is not attached to the tool (attached: {', '.join(attached) or 'none'})")
        if _PHYSICS:
            self._glue_cmd("detach", name)   # released: it falls under gravity from here
        self._diff(attached=[self._attached(name, CollisionObject.REMOVE)])

    def pose(self, name):
        """Where object `name` really is now: (xyz, rpy) in the base frame."""
        if name in self._sim_poses:   # MuJoCo's own pose (MoveIt's copy of a held object never slips)
            M = self._sim_poses[name]
            return tuple(float(v) for v in M[:3, 3]), _rpy(M[:3, :3])
        req = GetPlanningScene.Request()
        req.components.components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                                     | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        sc = _wait(self._get.call_async(req), 10).scene
        objs = {o.id: o for o in sc.world.collision_objects}
        for a in sc.robot_state.attached_collision_objects:
            objs[a.object.id] = a.object
        if name not in objs:
            raise UnknownObject(f"No object '{name}' (objects: {', '.join(sorted(objs)) or 'none'})")
        o = objs[name]
        M = _pose_matrix(o.pose) @ _pose_matrix(o.primitive_poses[0])
        if o.header.frame_id and o.header.frame_id != self.chain.base:  # attached: in the tool frame
            M = self._kin.fk(self._current()) @ M
        return tuple(float(v) for v in M[:3, 3]), _rpy(M[:3, :3])

    def _on_sim_poses(self, m):
        for s in m.free_joints:
            p = s.pose.pose
            M = _pose_matrix(p)
            self._sim_poses[s.name] = M

    def _gripper_cmd(self, position):
        from control_msgs.action import ParallelGripperCommand
        goal = ParallelGripperCommand.Goal()
        goal.command.name = [self.chain.gripper.leader]
        goal.command.position = [float(position)]
        self._run(self._gripper, goal, 30)
        self._settle()

    def gripper_width(self):
        """Opening left before the fingers are fully closed (m)."""
        g = self.chain.gripper
        if g is None:
            raise GripError("This robot has no gripper")
        self._current()
        return 2 * (g.upper - self._joints.get(g.leader, g.lower))

    def _tool_matrix(self):
        return self._kin.fk(self._current())

    def _object_matrix(self, name):
        if name in self._sim_poses:
            return self._sim_poses[name]
        xyz, rpy = self.pose(name)
        return _matrix(xyz, rpy)

    def release(self):
        """Open the gripper fully; a held object is let go (it falls or stays where it is)."""
        if self.chain.gripper is None:
            raise GripError("This robot has no gripper")
        self._gripper_cmd(self.chain.gripper.lower)
        if self._holding:
            name, _ = self._holding
            self._holding = None
            _, attached = self._scene()
            if name in attached:
                self._diff(attached=[self._attached(name, CollisionObject.REMOVE)])

    def grip(self, name=None):
        """Close the gripper on the object between the fingers; it is then carried with the tool."""
        g = self.chain.gripper
        if g is None:
            raise GripError("This robot has no gripper; use attach(name)")
        world, _ = self._scene()
        if name is not None and name not in world:
            raise UnknownObject(f"'{name}' is not a free object in the scene (objects: {', '.join(world) or 'none'})")
        obj = next((o for o in _meta()["objects"] if o["name"] == name), None)
        if obj is not None:
            T = self._tool_matrix()
            wide = grip_logic.fits(obj, T[:3, :3] @ np.asarray(g.closing_axis_tcp, float), 2 * (g.upper - g.lower))
            if wide is not None:
                raise GripError(f"'{name}' is {wide:.2f} m wide; the gripper opens to {2 * (g.upper - g.lower):.2f} m.")
        self._gripper_cmd(g.upper)
        if _PHYSICS and grip_logic.outcome(self.gripper_width(), 0.0) == "empty":
            raise GripError(f"The gripper closed on nothing: no object between the fingers at {_fmt(self.tool_pose()[0])}.")
        if name is None:
            if not world:
                raise GripError("Nothing to grip: the scene has no objects")
            name = min(world, key=self._gap)
        if not _PHYSICS and self._gap(name) > ATTACH_REACH:   # mock fingers touch nothing: check reach instead
            raise UnknownObject(f"'{name}' is {self._gap(name):.2f} m from the tool point; move the fingers around it "
                                f"first (within {ATTACH_REACH * 100:.0f} cm)")
        self._diff(attached=[self._attached(name, CollisionObject.ADD)])
        self._holding = (name, np.linalg.inv(self._tool_matrix()) @ self._object_matrix(name))

    def _slip_check(self):
        if not (_PHYSICS and self._holding):
            return
        name, before = self._holding
        now = np.linalg.inv(self._tool_matrix()) @ self._object_matrix(name)
        if grip_logic.slipped(before, now):
            self._holding = None
            m = _meta()["masses"].get(name, 0.0)
            raise GripError(f"'{name}' slipped out of the gripper while moving ({m:g} kg, "
                            f"grip force {self.chain.gripper.effort:g} N per finger).")

    def _load_run_objects(self):
        """Put the run's objects (workcell + SCENE, from sim_ctl) into MoveIt at their start poses."""
        adds = []
        for o in _meta()["objects"]:
            co = CollisionObject()
            co.id = o["name"]
            co.header.frame_id = self.chain.base
            co.pose = _pose(o["xyz"], o["rpy"])
            box = o["shape"] == "box"
            dims = o["size"] if box else (o["size"][1], o["size"][0])   # cylinder: (height, radius)
            co.primitives = [SolidPrimitive(type=SolidPrimitive.BOX if box else SolidPrimitive.CYLINDER,
                                            dimensions=[float(d) for d in dims])]
            co.primitive_poses = [_pose((0, 0, 0), (0, 0, 0))]
            co.operation = CollisionObject.ADD
            adds.append(co)
        if adds:
            self._diff(world=adds)

    def _physics_check(self, trajectory, code: int, elapsed: float):
        """After a motion in MuJoCo: a blown-up sim, or a joint the controller gave up on
        `elapsed` seconds into the trajectory."""
        if not _PHYSICS:
            return
        self._settle()
        names = self.joint_names
        pos = [self._joints.get(n, 0.0) for n in names]
        vel = [self._velocities.get(n, 0.0) for n in names]
        boom = sim_errors.blow_up(names, pos, vel, {j.name: j.child for j in self.chain.joints})
        if boom:
            raise MotionError(boom)
        if _ERRORS.get(code) == "CONTROL_FAILED" and trajectory.joint_trajectory.points:
            jt = trajectory.joint_trajectory
            times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in jt.points]
            now = dict(zip(jt.joint_names, sim_errors.planned_at(times, [list(p.positions) for p in jt.points], elapsed)))
            planned = [now.get(n, p) for n, p in zip(names, pos)]
            m = _meta()
            carrying = {k: m["masses"][k] for k in sorted(self._glued) if k in m["masses"]}
            hinge = {j.name: j.type != "prismatic" for j in self.chain.joints}
            raise MotionError(sim_errors.lag_message(names, planned, pos, m["efforts"], carrying, hinge))

    def reset(self):
        self._holding = None
        if self.chain.gripper is not None:
            self._gripper_cmd(self.chain.gripper.lower)
        self._clear_scene()
        self._load_run_objects()  # workcell + SCENE (sim_ctl.prepare); under MuJoCo the sim already holds them
        self.home()

    def _clear_scene(self):
        _, attached = self._scene()
        if attached:
            self._diff(attached=[self._attached(n, CollisionObject.REMOVE) for n in attached])
        world, _ = self._scene()
        removals = []
        for n in world:
            co = CollisionObject()
            co.id, co.operation = n, CollisionObject.REMOVE
            removals.append(co)
        if removals:
            self._diff(world=removals)

    def _on_joints(self, m):
        self._joints.update(zip(m.name, m.position))
        self._velocities.update(zip(m.name, m.velocity))
        self._stamp += 1

    def _settle(self):
        """Wait for a joint state that arrived after the motion finished. Under physics also wait
        until the arm is at rest: the controller reports success within tolerance while it still
        settles, and MoveIt rejects a next motion that starts from a stale pose."""
        seen, end = self._stamp, time.time() + 2
        while self._stamp == seen and time.time() < end:
            rclpy.spin_once(_node(), timeout_sec=0.05)
        while (_PHYSICS and time.time() < end
               and any(abs(self._velocities.get(n, 0.0)) > STILL for n in self.joint_names)):
            rclpy.spin_once(_node(), timeout_sec=0.05)

    # --- state -------------------------------------------------------------------------------
    def _current(self) -> np.ndarray:
        end = time.time() + 5
        while any(n not in self._joints for n in self.joint_names) and time.time() < end:
            rclpy.spin_once(_node(), timeout_sec=0.05)
        missing = [n for n in self.joint_names if n not in self._joints]
        if missing:
            raise PlanningFailed(f"no joint states received for {', '.join(missing)}")
        return np.array([self._joints[n] for n in self.joint_names])

    def tool_pose(self) -> tuple:
        T = self._kin.fk(self._current())
        return tuple(float(v) for v in T[:3, 3]), _rpy(T[:3, :3])

    def solutions(self, xyz, rpy) -> list:
        try:
            sols = self._kin.solve_all(_matrix(xyz, rpy), self._current())
        except NotImplementedError:
            raise PlanningFailed("exact IK is not available for this robot; use move_to and let MoveIt solve it")
        return [[float(v) for v in q] for q in sols]

    # --- motion ------------------------------------------------------------------------------
    def _check_reach(self, xyz, rpy):
        try:
            if not self._kin.solve_all(_matrix(xyz, rpy), self._current()):
                raise Unreachable(f"no joint solution puts the tool at {_fmt(xyz)} with orientation {_fmt(rpy)} "
                                  "(outside the reach or the joint limits)")
        except NotImplementedError:
            pass  # no closed form for this robot: MoveIt's own IK decides

    def _position_ik(self, xyz, tries: int = 8, seed=None) -> list:
        """Joint solutions that put the tool link at xyz with any orientation, computed on the model
        (damped least squares from `seed` (default: the current state) and random seeds); nearest to the seed first."""
        target = np.asarray(xyz, float)
        lo = np.array([j.lower if j.lower is not None else -math.pi for j in self.chain.joints])
        hi = np.array([j.upper if j.upper is not None else math.pi for j in self.chain.joints])
        rng = np.random.default_rng(0)
        cur = self._current() if seed is None else np.asarray(seed, float)
        sols = []
        for q in [cur] + [rng.uniform(lo, hi) for _ in range(tries - 1)]:
            q = np.clip(np.asarray(q, float), lo, hi)
            for _ in range(100):
                p = self._kin.fk(q)[:3, 3]
                err = target - p
                if np.linalg.norm(err) < 1e-4:
                    sols.append(q)
                    break
                J = np.empty((3, len(q)))
                for i in range(len(q)):
                    dq = np.zeros(len(q))
                    dq[i] = 1e-6
                    J[:, i] = (self._kin.fk(q + dq)[:3, 3] - p) / 1e-6
                step = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(3), err)
                q = np.clip(q + step * min(1.0, 0.3 / max(1e-9, float(np.abs(step).max()))), lo, hi)
        return sorted(sols, key=lambda s: float(np.linalg.norm(s - cur)))

    def _joint_frames(self, q) -> list:
        """Base-frame pose of each joint frame (before its own motion) at joint values q."""
        T, out = np.eye(4), []
        for j, qi in zip(self.chain.joints, q):
            T = T @ j.origin
            out.append(T.copy())
            ax = np.asarray(j.axis, float)
            M = np.eye(4)
            if j.type == "prismatic":
                M[:3, 3] = ax * qi
            elif j.type in ("revolute", "continuous"):
                k = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
                M[:3, :3] = np.eye(3) + math.sin(qi) * k + (1 - math.cos(qi)) * k @ k
            T = T @ M
        return out

    def tool_facing(self, direction, yaw=None) -> tuple:
        """rpy that points the tool (the flange, along the last joint's axis, away from the arm; with a
        gripper, its tool point's z) toward `direction` in the base frame, turning the home orientation
        as little as possible. yaw (radians): turn about that axis so the jaws close along
        (cos yaw, sin yaw) in the base x-y plane."""
        d = np.asarray(direction, float)
        if np.linalg.norm(d) < 1e-9:
            raise ValueError("tool_facing needs a non-zero direction")
        d = d / np.linalg.norm(d)
        q = _home(self.chain)
        T = self._kin.fk(q)
        R = T[:3, :3]
        if self.chain.gripper is not None:
            a = R[:, 2]                                                # the tool point's z is the approach
        else:
            a = R @ np.asarray(self.chain.joints[-1].axis, float)      # approach axis in the base frame
            # the axis may point either way: outward is away from the rest of the arm (the earlier joint frames)
            if sum(float(a @ (p - T[:3, 3])) for p in (F[:3, 3] for F in self._joint_frames(q)[:-1])) > 0:
                a = -a
        v, c = np.cross(a, d), float(a @ d)
        if np.linalg.norm(v) < 1e-9 and c > 0:
            turn = np.eye(3)
        elif np.linalg.norm(v) < 1e-9:                                 # opposite: half turn about any perpendicular
            u = np.cross(a, np.eye(3)[int(np.argmin(np.abs(a)))])
            u /= np.linalg.norm(u)
            turn = 2 * np.outer(u, u) - np.eye(3)
        else:
            vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
            turn = np.eye(3) + vx + vx @ vx * (1 / (1 + c))
        R = turn @ R
        if yaw is not None:
            closing = self.chain.gripper.closing_axis_tcp if self.chain.gripper is not None else (1.0, 0.0, 0.0)
            R = grip_logic.yaw_rotation(R, np.asarray(closing, float), float(yaw))
        return _rpy(R)

    def tool_down(self, yaw=None) -> tuple:
        """rpy with the tool pointing straight down (tool_facing((0, 0, -1), yaw))."""
        return self.tool_facing((0, 0, -1), yaw)

    def _run(self, client, goal, timeout):
        handle = _wait(client.send_goal_async(goal), 30)
        if not handle.accepted:
            raise PlanningFailed("MoveIt refused the motion request")
        return _wait(handle.get_result_async(), timeout).result

    def _error(self, code: int, what: str) -> RobotError:
        name = _ERRORS.get(code, str(code))
        if name in _UNREACHABLE:
            return Unreachable(f"{what} cannot be reached ({name})")
        if name in _COLLISION:
            world, attached = self._scene()
            return Collision(f"{what} collides with the robot or the scene ({name}); "
                             f"objects: {', '.join(world + attached) or 'none'}")
        if name in _NO_PATH:
            world, attached = self._scene()
            return PlanningFailed(f"{what}: MoveIt found no collision-free path ({name}). The target may be inside "
                                  "or touching an object or the robot itself; keep a few centimetres of clearance "
                                  "(the tool's own geometry extends past the tool frame); "
                                  f"objects: {', '.join(world + attached) or 'none'}")
        return PlanningFailed(f"{what}: MoveIt could not plan it ({name})")

    def _plan_execute(self, constraints: Constraints, what: str):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = GROUP
        req.num_planning_attempts = 5
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5
        req.start_state.is_diff = True
        req.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        t0 = time.time()
        res = self._run(self._move, goal, 120)
        self._physics_check(res.planned_trajectory, res.error_code.val, time.time() - t0 - res.planning_time)
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            raise self._error(res.error_code.val, what)
        self._settle()
        self._slip_check()

    def move_to(self, xyz, rpy=None):
        if rpy is None:
            cands = self._position_ik(xyz)
            if not cands:
                raise Unreachable(f"no joint solution puts the tool at {_fmt(xyz)} "
                                  "(outside the reach or the joint limits)")
            last = None
            for q in cands[:4]:
                try:
                    self.move_joints(list(q))
                    return
                except (Collision, PlanningFailed) as exc:
                    last = exc
            raise last
        self._check_reach(xyz, rpy)
        target = _pose(xyz, rpy)
        pc = PositionConstraint()
        pc.header.frame_id, pc.link_name, pc.weight = self.chain.base, self.chain.tip, 1.0
        pc.constraint_region.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[1e-3])]
        centre = Pose()
        centre.position = target.position
        centre.orientation.w = 1.0
        pc.constraint_region.primitive_poses = [centre]
        oc = OrientationConstraint()
        oc.header.frame_id, oc.link_name, oc.weight = self.chain.base, self.chain.tip, 1.0
        oc.orientation = target.orientation
        oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = oc.absolute_z_axis_tolerance = 1e-3
        self._plan_execute(Constraints(position_constraints=[pc], orientation_constraints=[oc]),
                           f"pose {_fmt(xyz)}")

    def _step_line(self, xyz, step: float = 0.02):
        """Arms with fewer than 6 joints cannot keep an orientation along a line: follow it in short
        collision-checked joint moves, each to the position solution nearest the previous one."""
        # ponytail: stops briefly every `step` metres; one stitched joint trajectory if smooth motion matters
        start = np.asarray(self.tool_pose()[0], float)
        end = np.asarray(xyz, float)
        n = max(1, int(math.ceil(np.linalg.norm(end - start) / step)))
        q = self._current()
        for k in range(1, n + 1):
            p = start + (end - start) * k / n
            sols = self._position_ik(p, seed=q)
            if not sols or np.abs(sols[0] - q).max() > 0.5:
                raise PlanningFailed(f"a straight line to {_fmt(xyz)} is only possible for {(k - 1) / n * 100:.0f}% "
                                     "of the way (the arm cannot follow it there); try move_to")
            q = sols[0]
            self.move_joints(list(q))

    def move_joints(self, q):
        q = [float(v) for v in q]
        if len(q) != len(self.joint_names):
            raise PlanningFailed(f"move_joints needs {len(self.joint_names)} values ({', '.join(self.joint_names)})")
        for v, j in zip(q, self.chain.joints):
            if j.lower is not None and not j.lower - 1e-6 <= v <= j.upper + 1e-6:
                raise Unreachable(f"{j.name} = {v:.3f} is outside its limits {j.lower:.3f} .. {j.upper:.3f}")
        cs = []
        for name, v in zip(self.joint_names, q):
            jc = JointConstraint()
            jc.joint_name, jc.position, jc.weight = name, v, 1.0
            jc.tolerance_above = jc.tolerance_below = 1e-3
            cs.append(jc)
        self._plan_execute(Constraints(joint_constraints=cs), f"joint target {_fmt(q)}")

    def home(self):
        self.move_joints(_home(self.chain))

    def move_line(self, xyz, rpy=None):
        if rpy is None and len(self.chain.joints) < 6:
            return self._step_line(xyz)
        if rpy is None:
            rpy = self.tool_pose()[1]                    # keep the current orientation along the line
        self._check_reach(xyz, rpy)
        req = GetCartesianPath.Request()
        req.header.frame_id = self.chain.base
        req.group_name = GROUP
        req.link_name = self.chain.tip
        req.waypoints = [_pose(xyz, rpy)]
        req.max_step = 0.005
        req.avoid_collisions = True
        req.start_state.is_diff = True
        for field in ("max_velocity_scaling_factor", "max_acceleration_scaling_factor"):
            if hasattr(req, field):
                setattr(req, field, 0.5)
        res = _wait(self._cartesian.call_async(req), 30)
        if res.fraction < 0.999:
            if res.error_code.val != MoveItErrorCodes.SUCCESS and res.fraction == 0.0:
                raise self._error(res.error_code.val, f"straight line to {_fmt(xyz)}")
            raise PlanningFailed(f"a straight line to {_fmt(xyz)} is only possible for {res.fraction * 100:.0f}% "
                                 "of the way (an obstacle, a joint limit or the edge of the reach is in the way); "
                                 "try move_to or a different path")
        t0 = time.time()
        out = self._run(self._exec, ExecuteTrajectory.Goal(trajectory=res.solution), 120)
        self._physics_check(res.solution, out.error_code.val, time.time() - t0)
        if out.error_code.val != MoveItErrorCodes.SUCCESS:
            raise self._error(out.error_code.val, f"straight line to {_fmt(xyz)}")
        self._settle()
        self._slip_check()
