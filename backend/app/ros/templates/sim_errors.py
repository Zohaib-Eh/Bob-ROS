"""Plain-language messages for physics failures (pure Python, tested on the host)."""
import math


def lag_message(names, planned, actual, efforts, carrying, hinge) -> str:
    """Name the joint furthest behind, what it carried and its motor limit."""
    i = max(range(len(names)), key=lambda k: abs(planned[k] - actual[k]))
    n, err = names[i], abs(planned[i] - actual[i])
    rotary = hinge.get(n, True)
    amount = f"{math.degrees(err):.0f}°" if rotary else f"{err * 1000:.0f} mm"
    unit = "N·m" if rotary else "N"
    load = "".join(f" while carrying {k} ({v:g} kg)" for k, v in carrying.items())
    fix = "or use a lighter load." if carrying else "or check what it hit."
    return f"{n} fell {amount} behind{load}. Motor limit {efforts.get(n, 0):g} {unit}: raise Max torque in Simulate, {fix}"


def blow_up(names, positions, velocities, child_link, limit=50.0):
    """The blown-up message for the worst joint (NaN, or faster than `limit`), else None."""
    bad = [k for k in range(len(names)) if not math.isfinite(positions[k]) or not math.isfinite(velocities[k])]
    if not bad:
        bad = [k for k in range(len(names)) if abs(velocities[k]) > limit]
    if not bad:
        return None
    k = max(bad, key=lambda j: abs(velocities[j]) if math.isfinite(velocities[j]) else math.inf)
    return f"Simulation blew up: check {child_link.get(names[k], names[k])}'s mass and inertia."
