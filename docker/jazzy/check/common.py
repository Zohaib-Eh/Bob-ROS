"""Helpers shared by the container checks."""
import importlib
import math
import time

import numpy as np
import rclpy


def load_kinematics(robot: str):
    return importlib.import_module(f"{robot}_kinematics").load()


def quat(R) -> tuple[float, float, float, float]:
    """(x, y, z, w) from a rotation matrix."""
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return x, y, z, w


def set_pose(msg, T) -> None:
    """Fill a geometry_msgs/Pose from a 4x4."""
    msg.position.x, msg.position.y, msg.position.z = (float(v) for v in T[:3, 3])
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = quat(T[:3, :3])


def pose_error(a, b) -> float:
    dp = float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
    c = (np.trace(a[:3, :3].T @ b[:3, :3]) - 1) / 2
    return max(dp, math.acos(max(-1.0, min(1.0, c))))


def same_angles(a, b, tol) -> bool:
    d = np.remainder(np.asarray(a) - np.asarray(b) + math.pi, 2 * math.pi) - math.pi
    return bool(np.all(np.abs(d) <= tol))


def wait(node, future, timeout: float):
    end = time.time() + timeout
    while not future.done() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not future.done():
        raise TimeoutError("no reply")
    return future.result()


def report(name: str, ok: bool, detail: str) -> bool:
    print(f"{'PASS' if ok else 'FAIL'} {name}: {detail}", flush=True)
    return ok
