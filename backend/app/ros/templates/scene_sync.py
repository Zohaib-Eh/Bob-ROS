"""Container node: keeps MoveIt's planning scene on the simulated object poses, so MoveIt plans
around where objects really are and the 3D view shows them. Carried (glued) objects are skipped:
MoveIt already moves them with the tool. The robot's base link sits at MuJoCo's world origin."""
import math

TOPIC = "/sim/free_joint_states"  # FreeJointStatePublisherPlugin, configured by sim_ctl.plugins_yaml


def moved(poses, last, skip, tol=0.002):
    """Objects whose (x, y, z, qx, qy, qz, qw) moved more than tol metres or 0.01 rad since last."""
    out = {}
    for name, p in poses.items():
        if name in skip:
            continue
        q = last.get(name)
        dot = abs(sum(a * b for a, b in zip(p[3:], q[3:]))) if q else 0.0
        if q is None or math.dist(p[:3], q[:3]) > tol or 2 * math.acos(min(1.0, dot)) > 0.01:
            out[name] = p
    return out


def main():
    import json
    from pathlib import Path

    import rclpy
    from moveit_msgs.msg import CollisionObject, PlanningScene
    from moveit_msgs.srv import ApplyPlanningScene
    from mujoco_ros2_control_msgs.msg import FreeJointStateArray
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String

    robot = next(p.name[:-len("_moveit_config")] for p in Path("/ws/src").glob("*_moveit_config"))
    base = __import__(f"{robot}_kinematics").load().chain.base
    rclpy.init()
    n = rclpy.create_node("cad_to_urdf_scene_sync")
    apply = n.create_client(ApplyPlanningScene, "/apply_planning_scene")
    state = {"last": {}, "glued": set(), "poses": {}}

    def on_glued(m):
        state["glued"] = set(filter(None, m.data.split(",")))

    def on_poses(m):
        state["poses"] = {s.name: (s.pose.pose.position.x, s.pose.pose.position.y, s.pose.pose.position.z,
                                   s.pose.pose.orientation.x, s.pose.pose.orientation.y,
                                   s.pose.pose.orientation.z, s.pose.pose.orientation.w)
                          for s in m.free_joints}

    def tick():
        meta = Path("/tmp/sim/meta.json")
        movable = set(json.loads(meta.read_text())["masses"]) if meta.exists() else set()
        todo = moved({k: v for k, v in state["poses"].items() if k in movable}, state["last"], state["glued"])
        if not todo or not apply.service_is_ready():
            return
        s = PlanningScene(is_diff=True)
        s.robot_state.is_diff = True
        for name, p in todo.items():
            co = CollisionObject(id=name, operation=CollisionObject.MOVE)
            co.header.frame_id = base
            (co.pose.position.x, co.pose.position.y, co.pose.position.z,
             co.pose.orientation.x, co.pose.orientation.y, co.pose.orientation.z, co.pose.orientation.w) = p
            s.world.collision_objects.append(co)
        apply.call_async(ApplyPlanningScene.Request(scene=s))
        state["last"].update(todo)

    n.create_subscription(String, "/sim/glued", on_glued,
                          QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    n.create_subscription(FreeJointStateArray, TOPIC, on_poses, 10)
    n.create_timer(0.1, tick)
    rclpy.spin(n)


if __name__ == "__main__":
    main()
