"""map -> odom for a mobile base, so that map -> base_link is where the base truly is.

Under MuJoCo the truth is the base body's pose (cad_to_urdf_glue publishes it on /sim/base_pose); the wheel
odometry (diff_drive_controller's odom -> base_link) drifts as wheels slip, and this transform takes the drift
out: map -> odom = truth * odom^-1. That is perfect localization; on a real robot AMCL or motion capture takes
this node's place. Without it (mock hardware) the base is where its odometry says, at floor height.
Usage: python3 localization.py <base link> <z of the base frame over the floor>"""
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def matrix(xyz, R) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, xyz
    return T


def map_to_odom(T_map_base, T_odom_base) -> np.ndarray:
    return T_map_base @ np.linalg.inv(T_odom_base)


class Localization(Node):
    def __init__(self, base: str, z0: float):
        super().__init__("localization")
        self.base, self.z0, self.truth = base, z0, None
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.out = TransformBroadcaster(self)
        self.create_subscription(PoseStamped, "/sim/base_pose", self.on_truth, 10)
        self.create_timer(0.02, self.publish)

    def on_truth(self, m):
        p, q = m.pose.position, m.pose.orientation
        self.truth = matrix([p.x, p.y, p.z], Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix())

    def publish(self):
        try:
            t = self.tf.lookup_transform("odom", self.base, Time()).transform
        except Exception:   # no odometry yet: the base controller is (re)starting
            return
        T_ob = matrix([t.translation.x, t.translation.y, t.translation.z],
                      Rotation.from_quat([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]).as_matrix())
        T_mb = self.truth if self.truth is not None else matrix([0, 0, self.z0], np.eye(3)) @ T_ob
        T = map_to_odom(T_mb, T_ob)
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = "map", "odom"
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = (float(v) for v in T[:3, 3])
        x, y, z, w = Rotation.from_matrix(T[:3, :3]).as_quat()
        msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = x, y, z, w
        self.out.sendTransform(msg)


def main():
    rclpy.init()
    rclpy.spin(Localization(sys.argv[1], float(sys.argv[2])))


if __name__ == "__main__":
    main()
