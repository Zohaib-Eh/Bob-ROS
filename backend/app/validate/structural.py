from app.models import Link, Joint


def validate_structure(links: list[Link], joints: list[Joint]) -> list[str]:
    errors: list[str] = []
    link_ids = {link.id for link in links}
    children_of_joint = {joint.child_link_id for joint in joints}
    parent_by_child = {joint.child_link_id: joint.parent_link_id for joint in joints}
    joints_by_child: dict[str, list[str]] = {}
    for joint in joints:
        joints_by_child.setdefault(joint.child_link_id, []).append(joint.id)
    for child, joint_ids in joints_by_child.items():
        if len(joint_ids) > 1:
            errors.append(
                f"Link '{child}' has more than one parent joint "
                f"({', '.join(joint_ids)}); a URDF tree allows exactly one parent per link."
            )

    roots = [link.id for link in links if link.id not in children_of_joint]
    if len(roots) == 0:
        errors.append("No root link found (every link has a parent joint).")
    elif len(roots) > 1:
        errors.append(
            f"Multiple root links found ({', '.join(sorted(roots))}); "
            "a URDF tree must have exactly one root."
        )

    for joint in joints:
        if joint.parent_link_id not in link_ids:
            errors.append(
                f"Joint {joint.id} references unknown parent link "
                f"'{joint.parent_link_id}'."
            )
        if joint.child_link_id not in link_ids:
            errors.append(
                f"Joint {joint.id} references unknown child link "
                f"'{joint.child_link_id}'."
            )

    for link in links:
        visited: set[str] = set()
        current = link.id
        while current in parent_by_child:
            if current in visited:
                errors.append(
                    f"Cycle detected in kinematic tree involving link '{current}'."
                )
                break
            visited.add(current)
            current = parent_by_child[current]

    non_leaf_ids = {joint.parent_link_id for joint in joints}
    for link in links:
        if link.id in non_leaf_ids:
            inertia = link.inertia or [0.0] * 6
            if all(abs(v) < 1e-12 for v in inertia[:3]):
                errors.append(
                    f"Link '{link.id}' is not a leaf but has zero inertia; "
                    "every non-leaf link must have nonzero inertia."
                )

    for link in links:
        if link.mass_kg is None or link.mass_kg <= 0:
            errors.append(
                f"Link '{link.id}' has zero or missing mass "
                f"(mass_kg={link.mass_kg}); a link must have positive mass."
            )

    return errors
