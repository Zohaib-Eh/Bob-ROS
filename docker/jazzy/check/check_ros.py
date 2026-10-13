"""Container check: controllers active, TF reaches the tip, MoveIt plans and executes to a pose.
Usage: python3 check_ros.py <robot>   (demo.launch.py must be running)"""
import math
import sys
import time

import numpy as np
import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

from common import load_kinematics, pose_error, quat, report, set_pose, wait

CONTROLLERS = ("arm_controller", "joint_state_broadcaster")


def main(robot: str) -> int:
    kin = load_kinematics(robot)
    chain = kin.chain
    rclpy.init()
    node = Node("cad_to_urdf_check")
    ok = True

    cm = node.create_client(ListControllers, "/controller_manager/list_controllers")
    states, end = {}, time.time() + 60
    while time.time() < end:
        if cm.wait_for_service(timeout_sec=1.0):
            states = {c.name: c.state for c in wait(node, cm.call_async(ListControllers.Request()), 5).controller}
            if all(states.get(n) == "active" for n in CONTROLLERS):
                break
        time.sleep(1.0)
    ok &= report("controllers active", all(states.get(n) == "active" for n in CONTROLLERS), str(states))

    buf = Buffer()
    TransformListener(buf, node)
    found, end = False, time.time() + 15
    while not found and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        found = buf.can_transform(chain.base, chain.tip, Time())
    ok &= report("TF base to tip", found, f"{chain.base} -> {chain.tip}")

    q = []
    for i, j in enumerate(chain.joints):
        v = 0.3 if i % 2 == 0 else -0.3
        q.append(v if j.lower is None else min(max(v, j.lower), j.upper))
    target = kin.fk(q)

    goal = MoveGroup.Goal()
    req = goal.request
    req.group_name = "arm"
    req.num_planning_attempts = 5
    req.allowed_planning_time = 10.0
    req.max_velocity_scaling_factor = 0.5
    req.max_acceleration_scaling_factor = 0.5
    pc = PositionConstraint()
    pc.header.frame_id, pc.link_name, pc.weight = chain.base, chain.tip, 1.0
    pc.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[1e-3]))
    centre = Pose()
    set_pose(centre, target)
    centre.orientation.x = centre.orientation.y = centre.orientation.z = 0.0
    centre.orientation.w = 1.0
    pc.constraint_region.primitive_poses.append(centre)
    oc = OrientationConstraint()
    oc.header.frame_id, oc.link_name, oc.weight = chain.base, chain.tip, 1.0
    oc.orientation.x, oc.orientation.y, oc.orientation.z, oc.orientation.w = quat(target[:3, :3])
    oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = oc.absolute_z_axis_tolerance = 1e-3
    req.goal_constraints.append(Constraints(position_constraints=[pc], orientation_constraints=[oc]))
    goal.planning_options.plan_only = False

    ac = ActionClient(node, MoveGroup, "/move_action")
    code = None
    if ac.wait_for_server(timeout_sec=30):
        handle = wait(node, ac.send_goal_async(goal), 30)
        code = wait(node, handle.get_result_async(), 120).result.error_code.val
    ok &= report("MoveIt plan and execute", code == 1, f"error code {code}")

    latest = {}
    node.create_subscription(JointState, "/joint_states", lambda m: latest.update(zip(m.name, m.position)), 10)
    end = time.time() + 3
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    q_now = [latest.get(j.name, math.nan) for j in chain.joints]
    err = pose_error(kin.fk(q_now), target) if not np.isnan(q_now).any() else math.inf
    ok &= report("joint_states at target", err < 2e-3, f"pose error {err:.2e}")

    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
