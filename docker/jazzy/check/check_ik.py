"""Container check: EAIK returns every IK solution and each reproduces its pose; TRAC-IK agrees.
When EAIK has no closed form for the robot, its checks print SKIP with EAIK's reason and only
TRAC-IK is checked against FK.
Usage: python3 check_ik.py <robot>   (demo.launch.py must be running for /compute_ik)"""
import importlib
import math
import sys

import numpy as np
import rclpy
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node

from common import load_kinematics, pose_error, report, same_angles, set_pose, wait

POSES = 200
MATCH_RAD = 5e-2  # TRAC-IK stops at ~1e-5 m pose error: near singular poses that is up to ~1e-2 rad; distinct branches sit much farther apart


def main(robot: str) -> int:
    kin = load_kinematics(robot)
    chain = kin.chain
    home = importlib.import_module(f"{robot}_kinematics._chain").home(chain)
    names = [j.name for j in chain.joints]
    rng = np.random.default_rng(1)
    rclpy.init()
    node = Node("cad_to_urdf_ik_check")
    ik = node.create_client(GetPositionIK, "/compute_ik")
    if not ik.wait_for_service(timeout_sec=60):
        report("TRAC-IK service", False, "/compute_ik not available")
        return 1

    eaik_why = None
    counts, worst, missing, trac_ok, trac_match, trac_far = [], 0.0, 0, 0, 0, 0.0
    for _ in range(POSES):
        q = np.array([rng.uniform(j.lower, j.upper) if j.lower is not None else rng.uniform(-math.pi, math.pi)
                      for j in chain.joints])
        pose = kin.fk(q)
        sols = []
        if eaik_why is None:
            try:
                sols = kin.solve_all(pose)
            except NotImplementedError as exc:
                eaik_why = str(exc)
        if eaik_why is None:
            counts.append(len(sols))
            worst = max([worst] + [pose_error(kin.fk(s), pose) for s in sols])
            if not any(same_angles(s, q, 1e-4) for s in sols):
                missing += 1

        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name, r.ik_link_name, r.avoid_collisions = "arm", chain.tip, False
        r.timeout.nanosec = 50_000_000
        r.pose_stamped.header.frame_id = chain.base
        set_pose(r.pose_stamped.pose, pose)
        r.robot_state.joint_state.name = names
        r.robot_state.joint_state.position = [float(v) for v in home]
        res = wait(node, ik.call_async(req), 5)
        if res.error_code.val == 1:
            got = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
            qt = np.array([got[n] for n in names])
            if pose_error(kin.fk(qt), pose) < 1e-4:
                trac_ok += 1
                if sols:
                    d = min(float(np.abs(np.remainder(qt - s + math.pi, 2 * math.pi) - math.pi).max()) for s in sols)
                    trac_far = max(trac_far, d)
                    trac_match += d <= MATCH_RAD

    ok = True
    if eaik_why is None:
        ok &= report("EAIK finds the sampled pose's joints", missing == 0, f"missing {missing}/{POSES}")
        ok &= report("EAIK solutions reproduce their pose", worst <= 1e-6 and min(counts) >= 1,
                     f"worst error {worst:.1e}; solutions per pose min {min(counts)} max {max(counts)} "
                     f"mean {np.mean(counts):.1f}")
    else:
        print(f"SKIP EAIK: {eaik_why}", flush=True)
    ok &= report("TRAC-IK success rate", trac_ok >= 0.95 * POSES, f"{trac_ok}/{POSES}")
    if eaik_why is None:
        ok &= report("TRAC-IK answer is one of EAIK's", trac_match == trac_ok,
                     f"{trac_match}/{trac_ok} within {MATCH_RAD} rad; farthest {trac_far:.1e} rad")
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
