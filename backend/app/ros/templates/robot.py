"""Task library for __ROBOT__: the functions LLM-written programs call. MoveIt plans every motion.
Units: metres and radians, in the robot's base frame."""
import importlib
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject, Constraints, JointConstraint,
                             MoveItErrorCodes, OrientationConstraint, PlanningScene,
                             PlanningSceneComponents, PositionConstraint)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPlanningScene
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

ROBOT = "__ROBOT__"
GROUP = "arm"
_kin = importlib.import_module(f"{ROBOT}_kinematics")
_home = importlib.import_module(f"{ROBOT}_kinematics._chain").home
_ERRORS = {v: k for k, v in vars(MoveItErrorCodes).items() if k.isupper() and isinstance(v, int)}
_UNREACHABLE = {"NO_IK_SOLUTION", "INVALID_GOAL_CONSTRAINTS", "GOAL_CONSTRAINTS_VIOLATED"}
_NO_PATH = {"FAILURE", "PLANNING_FAILED", "TIMED_OUT"}
_COLLISION = {"GOAL_IN_COLLISION", "START_STATE_IN_COLLISION", "INVALID_MOTION_PLAN",
              "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE"}


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
        self._stamp = 0
        n.create_subscription(JointState, "/joint_states",
                              self._on_joints, 10)
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

    def add_box(self, name, size_xyz, xyz, rpy=(0, 0, 0)):
        self._add(name, SolidPrimitive.BOX, size_xyz, xyz, rpy)

    def add_cylinder(self, name, radius, height, xyz, rpy=(0, 0, 0)):
        self._add(name, SolidPrimitive.CYLINDER, (height, radius), xyz, rpy)

    def _attached(self, name, op) -> AttachedCollisionObject:
        a = AttachedCollisionObject()
        a.link_name = self.chain.tip
        a.object.id = name
        a.object.operation = op
        a.touch_links = [self.chain.tip]
        return a

    def attach(self, name):
        world, attached = self._scene()
        if name not in world:
            raise UnknownObject(f"'{name}' is not a free object in the scene "
                                f"(objects: {', '.join(world + attached) or 'none'})")
        self._diff(attached=[self._attached(name, CollisionObject.ADD)])

    def detach(self, name):
        _, attached = self._scene()
        if name not in attached:
            raise UnknownObject(f"'{name}' is not attached to the tool (attached: {', '.join(attached) or 'none'})")
        self._diff(attached=[self._attached(name, CollisionObject.REMOVE)])

    def reset(self):
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
        self.home()

    def _on_joints(self, m):
        self._joints.update(zip(m.name, m.position))
        self._stamp += 1

    def _settle(self):
        """Wait for a joint state that arrived after the motion finished."""
        seen, end = self._stamp, time.time() + 2
        while self._stamp == seen and time.time() < end:
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

    def tool_down(self) -> tuple:
        """rpy that points the tool's approach axis (the last joint's axis) straight down, turning the
        home orientation as little as possible."""
        q = _home(self.chain)
        T = self._kin.fk(q)
        R = T[:3, :3]
        a = R @ np.asarray(self.chain.joints[-1].axis, float)          # approach axis in the base frame
        # the axis may point either way: outward is away from the rest of the arm (the earlier joint frames)
        if sum(float(a @ (p - T[:3, 3])) for p in (F[:3, 3] for F in self._joint_frames(q)[:-1])) > 0:
            a = -a
        target = np.array([0.0, 0.0, -1.0])
        v, c = np.cross(a, target), float(a @ target)
        if np.linalg.norm(v) < 1e-9:
            turn = np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
        else:
            vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
            turn = np.eye(3) + vx + vx @ vx * (1 / (1 + c))
        return _rpy(turn @ R)

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
        res = self._run(self._move, goal, 120)
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            raise self._error(res.error_code.val, what)
        self._settle()

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
        out = self._run(self._exec, ExecuteTrajectory.Goal(trajectory=res.solution), 120)
        if out.error_code.val != MoveItErrorCodes.SUCCESS:
            raise self._error(out.error_code.val, f"straight line to {_fmt(xyz)}")
        self._settle()
