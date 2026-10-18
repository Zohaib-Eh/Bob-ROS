# File ▸ Import — Design (as built)

Date: 2026-10-18. Status: built, on branch `worktree-file-import`.

## What the user gets

| You have | Where | What happens |
|---|---|---|
| A URDF (`.urdf`), or a zip of a URDF and its meshes, including this app's own export | File ▸ Import URDF / robot zip… | The file replaces the current robot, after a confirm. It opens on Connect with its joints in place. |
| A finished project (`project.json`) | File ▸ Open project… | Already existed before this work. |
| A workcell from another robot or computer | Simulate ▸ Workcell ▸ Save… / Load… | Save downloads `workcell.yaml` plus its fixture meshes as a zip. Load takes that zip or a bare yaml and replaces the cell. |

## URDF import (`backend/app/importers/urdf_importer.py`, `POST /import/urdf`)

- **Parts.** Each link becomes one part made of its `<visual>` shapes. If a link has no visual, its `<collision>` shapes are used. Meshes (STL, OBJ, DAE) and boxes, cylinders and spheres all work.
- **Coordinates.** Parts are placed in world coordinates at q = 0, because the app keeps link frames and joint axes in world coordinates.
- **Mesh paths.** Paths resolve inside the zip the way ROS would: relative to the URDF first, then by the longest tail of the `package://` path that names exactly one file. That tail rule handles example-robot-data's `ur3/`, `ur5/` and `ur10/` meshes, which share file names.
- **Mesh units.** The mesh `scale` attribute is applied. So is the COLLADA `<unit>`, which ROS applies and trimesh ignores. `pycollada` was added to read `.dae` files.
- **Joints.** Joint types, limits and efforts (as `max_effort`) are kept. `floating` and other types become `fixed`, with a warning.
- **Links left out.** A link with no shape on a fixed joint (world, base_footprint, tool0, this app's `tcp`) is left out, and its children hang off its parent. A shapeless root is left out while it has exactly one fixed child.
- **Placeholders.** A shapeless link on a moving joint gets a 1 cm cube.
- **Mass.** The URDF mass becomes the link's mass override. Inertia is recomputed from the shape at that mass. The app has no inertia override; add one if fidelity needs the file's exact tensor.
- **Gripper.** A prismatic joint that mimics another prismatic joint on the same parent becomes the gripper.
- **Robot name.** The robot name comes from `<robot name>`.
- **No CAD faces.** The parts have no CAD faces, so joint detection and Mount snapping don't work on them. The import says so in the notify banner, together with every other warning.
- **Limits and errors.** An upload that unpacks to more than 1 GB is refused. Xacro is not supported: the error asks the user to run xacro first.

## Verified

- **Round trip.** This app's export of the six-axis-plus-gripper test robot imports back as the same links, joints, axes, limits and gripper.
- **Real robots.** Three outside robots were imported, then exported again:

  | Robot | Links | Container check |
  |---|---|---|
  | Panda (example-robot-data) | 11, gripper detected | 12 PASS, 2 FAIL |
  | UR (example-robot-data) | 8 | not run |
  | Fanuc M-710iC | 7 | 17 PASS, 0 FAIL |

  The Panda's two fails are its 7 joints: it has no exact IK, so the two programs that need it fail.
- **Browser.** Importing the Panda zip shows the arm, its joints and the gripper (80 mm open). Workcell Save, then Load, round-trips.
