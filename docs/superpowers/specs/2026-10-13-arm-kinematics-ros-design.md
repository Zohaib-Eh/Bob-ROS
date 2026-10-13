# Arm Kinematics and ROS 2 Packages — Design

**Date:** 2026-10-13
**Builds on:** the export package (`backend/app/export/urdf_writer.py`) and the collision sweep (`backend/app/validate/sweep.py`, spec `2026-10-08-collision-sweep-design.md`).
**Scope:** project 1 of the roadmap below. A backend module plus a CLI that turn an exported robot zip into a ROS 2 Jazzy workspace with a MoveIt config and an all-solutions IK package, verified in a Docker container. No UI change.

## 1. Purpose and success

The product goal is "bolt.new for robots": the user uploads a CAD model, the app makes the URDF, then a chat on one side writes a ROS 2 program while the robot runs it in simulation on the other side; the result ships as a container for the real robot. This project produces the robot setup half of that ROS workspace: the files nobody should hand-write (SRDF, kinematics, controllers, launch), generated deterministically from the URDF, readable by the user in the workspace file tree.

Success criteria:

- `python -m app.ros.cli irb1300.zip -o ws/` writes `ws/src/` with `irb1300` (description), `irb1300_moveit_config` and `irb1300_kinematics`.
- In a ROS 2 Jazzy container, `colcon build` of that workspace succeeds.
- `demo.launch.py` on mock hardware, headless: both controllers report `active`, the TF tree reaches the tip link from the base, and MoveIt plans and executes a move to a reachable pose; MoveIt reports success and `/joint_states` reaches the target.
- On 200 random reachable IRB 1300 poses, every EAIK solution reproduces its pose through the numpy FK within 1e-6 m / 1e-6 rad, and TRAC-IK's answer matches one of EAIK's solutions.
- A branching or wheeled robot is declined with a one-sentence reason and no partial output.

## 2. Roadmap (context, not scope)

1. **Arm kinematics and ROS 2 packages** — this spec.
2. Simulation container: Gazebo and/or MuJoCo, headless, joint states streamed to the browser viewer (not video). Adds convex collision shapes (primitives where they fit, CoACD convex decomposition elsewhere; MuJoCo otherwise hulls concave links), MJCF export, and a "stable after spawn" physics check.
3. Workspace UI: prompt on one side, live robot plus file tree on the other.
4. Prompt to program: an LLM writes MoveIt task code into `<robot>_tasks/`, run in simulation. Intent-file pattern: the LLM edits a small intent file (limits, materials, tasks), deterministic generators rebuild everything else; it never edits generated config.
5. Real hardware: vendor drivers (e.g. ABB via `abb_ros2`), calibration options, simulation-first and explicit confirmation before the first real motion.
6. Mobile bases (differential, skid-steer, mecanum/omni, Ackermann; via `ros2_control` drive controllers), then mobile manipulators.

Order update (2026-10-13): prompt to program (4) comes next, on the existing mock-hardware MoveIt setup with joint states streamed to the browser viewer; physics simulation (2) follows. Before real hardware (5): snap revolute hinge lines to round features (bores, bosses, housings) when several of different sizes share one line, and mark joints whose hinge is only estimated so the user checks them in the joint tool.

Later, unordered: native CAD mates as input (Onshape API first, then SolidWorks); grippers with mimic joints and closed loops; part classes (fasteners, bearings) for auto-grouping; MuJoCo RL; cross-simulator comparison.

Competitive notes (2026-10-13): closest tools are Drift (prompt → ROS 2 sim workspace, no CAD import, no hardware), Jointly and arugoa/cad-to-urdf (CAD → URDF/MJCF/USD), Trener Robotics (natural-language programming of commercial arms only). The gap is custom robots with no existing URDF; the edge is correctness without hand fixes (sweep-derived limits, verified kinematics) and local processing of private CAD.

## 3. Decisions (from the design conversation)

| Topic | Decision |
|---|---|
| Robots | Serial arms only. Branching, wheeled, planar/floating/spherical-joint robots are declined with a reason. Mobile is project 6. |
| ROS | ROS 2 Jazzy in Docker (host is Windows 11). |
| Form | Backend module `backend/app/ros/` plus CLI. App export UI integration later. |
| Generation | Python writes every config file directly (no MoveIt Setup Assistant, no per-robot template). |
| MoveIt IK plugin | TRAC-IK (`trac_ik_kinematics_plugin`, Jazzy binary 2.0.2), `solve_type: Distance`, 50 ms timeout. Commented pick_ik (Jazzy binary 1.1.3) alternative. |
| All-solutions IK | EAIK (BSD-3, pip, reads URDF): exact closed-form for any 6R, 5R and 1–4R arm. Wrapped in a generated `<robot>_kinematics` Python package for task programs. |
| Simulator hardware | `ros2_control` xacro takes a `hardware` argument. Project 1 ships `mock` only; `gz` and `mujoco` values come with project 2, where they are tested. |
| Launch | Own `demo.launch.py` (robot_state_publisher, ros2_control_node, controller spawners, move_group). No RViz: the browser viewer replaces it in project 3, and `moveit_configs_utils`' stock demo launch needs an RViz config we would otherwise have to generate. |
| File readability | Every generated file has a header comment saying it is generated, that regenerating overwrites it, and that user code goes in `<robot>_tasks/`. Detection results are written as comments. |

### Why not OPW (researched 2026-10-13)

The plan started with the OPW closed-form plugin for spherical-wrist 6-axis arms. `moveit_opw_kinematics_plugin` has no ROS 2 release (ROS 1/kinetic only, last update 2020). EAIK (2025, built on IK-Geo) covers OPW's robots and more general ones, derives the solution itself from a URDF, and removes the need to hand-extract OPW parameters. It has no MoveIt plugin, so MoveIt uses TRAC-IK and EAIK serves the all-solutions role. A MoveIt plugin wrapping EAIK is deferred until TRAC-IK proves too slow or jumpy. EAIK installs from PyPI on Linux only, which is fine: it runs in the container, never on the Windows backend.

## 4. Export fix

`backend/app/export/urdf_writer.py` writes a ROS 1 `package.xml` (`catkin`) and no `CMakeLists.txt`, so ROS 2 cannot build the export.

- `package.xml` becomes format 3 with `ament_cmake` and `<export><build_type>ament_cmake</build_type></export>`.
- New `CMakeLists.txt` installs `urdf/` and `meshes/` to `share/<robot>/`.
- Mesh references already use `package://<robot>/meshes/<link>.stl`; unchanged.
- Joint names change from `joint_<uuid>` (hyphens; ugly in every generated file and risky as ROS parameter keys) to `<child link>_joint`, with characters outside `[A-Za-z0-9_]` replaced by `_` and a numeric suffix on collision.
- The CLI also rejects robot names that are not valid ROS package names (`^[a-z][a-z0-9_]*$`); the app allows uppercase and hyphens.

## 5. Backend module `backend/app/ros/`

| Unit | Job | Depends on |
|---|---|---|
| `chain.py` | Parse the URDF (stdlib XML). Root = base. Fixed joints merge into their parent link. Checks in this order: two or more `continuous` joints whose child subtrees have no further moving joints → wheels, decline; `planar`/`floating`/`spherical` joints → decline; more than one path of moving joints → branching, decline. A single `continuous` joint (e.g. a free-spinning flange) is allowed. `revolute` and `prismatic` joints must have limits. | stdlib |
| `fk.py` | numpy FK along the chain from URDF origins and axes. Reference for every IK check. | numpy |
| `collisions.py` | Load link STLs, sample 10,000 random joint configs within limits, classify link pairs: `Adjacent` (joined by a joint), `Default` (collide at `home`), `Never` (never collide in any sample). Reuses `_object` / `_collides` from `app/validate/sweep.py` (made public). | `fk`, python-fcl, trimesh |
| `moveit_config.py` | Write the three packages (§6) into a temp folder, then move into place only if everything succeeded. | all above |
| `cli.py` | `python -m app.ros.cli <zip> -o <dir>`. Unzip, find `<robot>/urdf/<robot>.urdf`, generate `<dir>/src/`. Exit non-zero with one sentence on error. | `moveit_config` |

`home` = every joint at 0, or at the nearest limit if 0 is outside its range.

## 6. Generated workspace

```
ws/src/
├── <robot>/                      description (export zip, §4)
├── <robot>_moveit_config/
│   ├── config/
│   │   ├── <robot>.srdf              group "arm" base→tip, state "home", disable_collisions
│   │   ├── kinematics.yaml           TRAC-IK; commented pick_ik line
│   │   ├── joint_limits.yaml         positions from URDF; vel 1 rad/s, acc 2 rad/s² "# set from datasheet"
│   │   ├── ros2_controllers.yaml     joint_state_broadcaster + arm_controller (JointTrajectoryController, position)
│   │   ├── moveit_controllers.yaml   FollowJointTrajectory → arm_controller
│   │   ├── initial_positions.yaml    home
│   │   ├── <robot>.ros2_control.xacro  hardware arg: mock (mock_components/GenericSystem)
│   │   └── <robot>.urdf.xacro        includes description URDF + ros2_control xacro
│   ├── launch/demo.launch.py     robot_state_publisher + ros2_control_node + spawners + move_group; no RViz
│   ├── CMakeLists.txt
│   └── package.xml
└── <robot>_kinematics/           ament_python
    ├── <robot>_kinematics/__init__.py
    │     solve_all(pose) -> list[np.ndarray]   EAIK, sorted by distance to `current` (optional arg)
    │     fk(q) -> 4x4 pose
    ├── setup.py
    └── package.xml
```

`<robot>_kinematics` loads the installed URDF. If EAIK's end frame differs from the chain tip, the fixed transform between them is applied inside the package. If EAIK cannot solve the geometry, `solve_all` raises `NotImplementedError("no closed-form solver for this geometry")`; MoveIt is unaffected.

## 7. Errors

The CLI writes nothing partial and exits non-zero with one sentence:

- Branching chain: "Robot has N moving branches; only single-arm robots are supported yet."
- Wheels: "Joint '<j>' looks like a wheel; mobile robots are not supported yet."
- Unsupported joint type: names the joint and type.
- Revolute or prismatic joint without limits: "Joint '<j>' has no limits; set them in the app and export again."
- No `<robot>/urdf/<robot>.urdf` in the zip: "Not a cad-to-urdf export."
- Robot name not a valid ROS package name: "Robot name '<n>' is not a valid ROS package name; use lowercase letters, digits and underscores, starting with a letter."

## 8. Testing

**Layer 1 — pytest (normal suite, Windows, no ROS):**

- `chain`: IRB 1300 → 6 joints, right base and tip; branching fixture and wheeled fixture declined with reason; fixed links merged.
- `fk`: 2-link fixture matches hand-computed poses; IRB 1300 at zero puts the tip where the URDF says.
- `collisions`: adjacent pairs skipped; overlapping-at-home pair → `Default`; a pair that never meets → `Never`.
- `moveit_config`: IRB 1300 generation; SRDF and YAML parse; joint names match the URDF; every file has the header; an error leaves no output folder.
- Export: `package.xml` is ament_cmake; `CMakeLists.txt` present; joint names are `<child>_joint`.

**Layer 2 — Docker (on request, `scripts/ros_check.sh <zip>`):**

`docker/jazzy/Dockerfile` from `ros:jazzy` plus MoveIt, `trac_ik_kinematics_plugin`, `ros2_control`, `ros2_controllers`, mock components, `pip install EAIK`. The script runs the CLI, `colcon build`, launches the demo headless, checks both controllers are `active` and the TF tree reaches the tip, plans and executes to a reachable pose, and runs the 200-pose IK check (§1). Prints pass/fail with solutions per pose, worst error, TRAC-IK success rate.

**Host risk:** the Windows host has about 1 GB free commit memory; MoveIt plus `colcon build` in WSL2 typically wants 4 GB or more. The plan's first task checks that the container runs here. Fallbacks: free memory, raise the WSL2 limit in `.wslconfig`, or run layer 2 on GitHub Actions.

## 9. Out of scope

Mobile bases, Gazebo/MuJoCo, app UI, container export product, LLM task code, real hardware and calibration, a MoveIt plugin wrapping EAIK, velocity/acceleration limits from datasheets.
