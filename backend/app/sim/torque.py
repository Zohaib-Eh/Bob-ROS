"""Torque each joint needs to move the arm, from MuJoCo's inverse dynamics."""
import math

import mujoco
import numpy as np

from app.ros.chain import MAX_ACCELERATION, MAX_VELOCITY


def required_efforts(model: mujoco.MjModel, samples: int = 500, seed: int = 0) -> dict[str, float]:
    """Joint name -> largest |torque| (N·m, or N for slides) needed over `samples` random poses within
    the joint limits (full circle for unlimited hinges), every joint moving at the generated limits
    (MAX_VELOCITY, MAX_ACCELERATION, random directions): gravity plus inertia, as a motor is sized.
    Holding still against gravity alone left a vertical axis with ~0 and unable to swing the arm."""
    m = model
    m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONSTRAINT  # no contacts/limits: dynamics only
    d = mujoco.MjData(m)
    rng = np.random.default_rng(seed)
    joints = [j for j in range(m.njnt) if m.jnt_type[j] in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE)]
    worst = {j: 0.0 for j in joints}
    for _ in range(samples):
        d.qvel[:] = 0
        d.qacc[:] = 0
        for j in joints:
            lo, hi = m.jnt_range[j] if m.jnt_limited[j] else (-math.pi, math.pi)
            d.qpos[m.jnt_qposadr[j]] = rng.uniform(lo, hi)
            d.qvel[m.jnt_dofadr[j]] = rng.choice((-1.0, 1.0)) * MAX_VELOCITY
            d.qacc[m.jnt_dofadr[j]] = rng.choice((-1.0, 1.0)) * MAX_ACCELERATION
        mujoco.mj_inverse(m, d)
        for j in joints:
            worst[j] = max(worst[j], abs(float(d.qfrc_inverse[m.jnt_dofadr[j]])))
    return {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j): v for j, v in worst.items()}


def suggest(required: dict[str, float]) -> dict[str, float]:
    """2x the need, rounded up to 2 significant figures, at least 1."""
    out = {}
    for name, need in required.items():
        x = 2 * need
        if x <= 1:
            out[name] = 1.0
            continue
        step = 10 ** (math.floor(math.log10(x)) - 1)
        out[name] = float(math.ceil(x / step - 1e-9) * step)
    return out
