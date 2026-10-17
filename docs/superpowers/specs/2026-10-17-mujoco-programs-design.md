# MuJoCo Programs (part C): Design

**Date:** 2026-10-17
**Builds on:**
- `2026-10-14-workbenches-mujoco-check-design.md`: parts A and B, the MJCF export and the hold check.
- `2026-10-13-program-workspace-design.md`: Program mode, `robot.py`, `runner.py`.

**Branch:** `mujoco-programs`, on top of `workbenches-mujoco` and the inertia fix.

**Scope:** programs drive a physically simulated arm, and the objects are physical too. The mock hardware goes away whenever a MuJoCo model exists.

## 1. Goal

Today a run uses mock hardware: every joint is instantly where it was told to be, and boxes are "ghost" planning objects that snap onto the tool. Part C makes a run behave like a real cell:
- The arm has gravity, motor limits, lag and overshoot.
- Boxes have weight, fall, tip and can be knocked.

Success criteria:
- On the irb1300, Program-mode runs execute in MuJoCo through the standard ros2_control trajectory follower, and the 3D view shows the simulated motion.
- A pick-and-place program works: `attach`, carry, `detach`, the box falls and settles, and `pose()` reports it within 1 cm of the target.
- An overweight box or a too-weak motor makes the move fail with a message that names the joint and the cause.
- Every run starts from the same world: the workcell plus the program's objects, with the arm at home.
- At least 4 of the 5 benchmark tasks (prompt-to-program spec §8) work on the irb1300 under physics, within 3 runs each.

## 2. Decisions

| Topic | Decision |
|---|---|
| Simulator hookup | `mujoco_ros2_control` (official ros-controls, Jazzy apt `ros-jazzy-mujoco-ros2-control`) replaces `mock_components/GenericSystem`. MoveIt → `arm_controller` (JointTrajectoryController) → MuJoCo position motors. Real hardware later only swaps the hardware plugin, for example `abb_ros2`. |
| Fidelity | Everything physical: arm and objects (user, 2026-10-17). Until grippers exist, `attach()` is a weld constraint from the object to the tool. |
| Objects | Declared before the run, never created mid-run. A real cell is set up before the program starts, and the sim loads a fixed scene. |
| Where objects live | Both: `<robot>_tasks/workcell.yaml`, shared by all programs, plus a `SCENE` list at the top of each program. They are merged per run, and duplicate names are an error. |
| Workcell editing | The file in the Code tab, editable by the user and by Claude when asked, shown in the 3D Preview. The drag-in-3D editor is next-but-one on the roadmap. |
| Glue | Our C++ plugin (`<robot>_sim_plugins`, a `MuJoCoROS2ControlPluginBase`). It holds one inactive weld per movable object and toggles `eq_active` in `pre_step`, recording the current relative pose first so the object doesn't jump. |
| Object sync | The free-joint pose publisher plugin feeds `scene_sync.py`, which writes object poses into MoveIt's planning scene. MoveIt plans around real positions, and the 3D view (which draws MoveIt's scene) shows them. |
| Reset per run | Generate `scene.xml`. If it is identical to the loaded one, call `reset_world`. Otherwise restart only the sim node (`ros2_control_node`) with the new scene. MoveIt stays up. |
| Speed | Real time (`sim_speed_factor` 1.0), headless. |
| No MJCF | Fall back to mock hardware, as today. The top bar shows "Physics off: <reason>". |

## 3. Container pieces

- **Docker image** (`docker/jazzy/Dockerfile`) gains `ros-jazzy-mujoco-ros2-control` and builds the glue plugin as part of the generated workspace.
- **`moveit_config.py`:** the `hardware` xacro arg gains `mujoco`. Its `<hardware>` block uses the `mujoco_ros2_control` plugin with `mujoco_model` set to `scene.xml` and `headless` true. The joint command interface is `position`; the state interfaces are `position`, `velocity` and `effort`. The launch picks `mujoco` when `<robot>/mjcf/<robot>.xml` exists, and `mock` otherwise.
- **`scene.py`** (new, `<robot>_program/`): validates and merges the objects, then writes `scene.xml`.
  - It includes the robot MJCF.
  - Each object becomes a body. Fixed objects are bodies with no joint. Movable ones have a free joint, mass, and optional friction.
  - Each movable object gets one `<weld>` equality from it to the tool link, inactive.
- **`sim_ctl.py`** (new): starts and restarts the sim node with a given scene, waits for `arm_controller` to be active (30 s timeout), and calls `reset_world`.
- **`scene_sync.py`** (new node, started with MoveIt): turns free-joint poses into planning-scene updates for movable objects. Objects attached to the tool are skipped, because MoveIt already carries them with the tool. Fixed objects are added once per reset.
- **Glue plugin:** two services, `~/attach` and `~/detach` (object name), plus `on_reset` that switches every weld off.

## 4. Objects and the program API

One format for `workcell.yaml` entries and `SCENE` items:

```yaml
- name: table
  shape: box               # box | cylinder
  size: [1.2, 0.8, 0.05]   # box: x y z (m); cylinder: [radius, height]
  xyz: [0.6, 0, 0.375]
  rpy: [0, 0, 0]           # optional
  fixed: true              # optional; bolted down, never moves, can't be attached
- name: part_a
  shape: box
  size: [0.05, 0.05, 0.05]
  xyz: [0.5, 0.1, 0.43]
  mass: 0.2                # kg; optional, default 700 kg/m³ × volume
  friction: 0.8            # optional, MuJoCo default otherwise
```

In a program: `SCENE = [dict(name="part_a", shape="box", size=(0.05, 0.05, 0.05), xyz=(0.5, 0.1, 0.43), mass=0.2)]`. The runner reads it with `ast.literal_eval` (no execution) before the run, so `SCENE` must be plain values: numbers, strings, lists, tuples and `dict(...)` or `{...}` with those. A computed value fails with "SCENE must be plain values (line N)".

Checks before the sim starts. Each failure is reported as `RESULT: failed: <message>`:
- unknown fields, a bad shape, sizes ≤ 0, mass ≤ 0, `mass` on a fixed object
- duplicate names between the workcell and `SCENE`
- an object overlapping the robot at home (MuJoCo contact check at home pose)

`robot.py` changes:
- `add_box` / `add_cylinder` raise `RobotError`: "Objects can't appear mid-run. Declare '<name>' in SCENE at the top of the program (or in workcell.yaml)." They stay importable so old programs fail with that message instead of an AttributeError.
- `attach(name)` keeps the reach check (2 cm), refuses `fixed` objects, then calls the plugin's `attach`. The MoveIt attached object is kept, so planning still accounts for the carried box.
- `detach(name)` calls the plugin's `detach` and removes the MoveIt attachment. The box falls under gravity.
- New `pose(name) -> (xyz, rpy)` returns the object's simulated pose.
- `reset()` performs the per-run reset (§2), then goes home.

Claude's brief is updated:
- Declare objects in `SCENE`.
- The workcell already exists; its objects are listed with sizes and positions.
- Physics is real, so check results with `pose()` instead of assuming them.

## 5. Errors

| Situation | What the user sees |
|---|---|
| Arm can't track (weak motor, heavy load, contact) | The JointTrajectoryController path or goal tolerance trips, and MoveIt aborts. `MotionError`: "joint2 fell 12° behind while carrying part_a (0.2 kg). Motor limit 150 N·m: raise Max torque in Simulate, or use a lighter load." The joint is the one with the largest error between `/joint_states` and the plan at abort. |
| Object ends up somewhere unexpected | No failure: it is a real outcome. `pose()` and the 3D view show it. |
| Sim blows up (NaN or runaway velocity) | Run fails: "Simulation blew up: check <link>'s mass and inertia" (same wording as the Physics card). The next run restarts the sim. |
| Scene won't load | Run fails before the program starts, with MuJoCo's message. |
| Sim restart exceeds 30 s | Run fails with the sim node's last 20 log lines. |
| No MJCF in the export | Mock hardware. The top bar shows "Physics off: <reason>" and objects behave as before. |

The top bar shows "Physics: MuJoCo" or "Physics off: <reason>" next to the ROS status.

## 6. Testing

**Task 1 (verify before building).** If any of these fail, stop and bring it back to the user:
- `ros-jazzy-mujoco-ros2-control` installs in the image, or else builds from source.
- The exported irb1300 MJCF loads with `position` actuators (the docs hint that motor-type actuators may be required; if so, the export adapts).
- Measure sim restart time. Find out whether `/mujoco_robot_description` can load a new scene without a restart.
- The glue plugin builds and can toggle a weld in `pre_step`.

Then:
- **Backend unit tests (no Docker):**
  - object validation and the workcell + `SCENE` merge
  - reading `SCENE` with `ast`
  - `scene.xml` that compiles in MuJoCo Python, with the right masses and welds inactive
  - the MoveIt-abort → `MotionError` message
- **MuJoCo Python tests (no ROS):**
  - the generated scene holds the home pose
  - a dropped box lands and settles
  - a welded box follows the tool, and releasing it keeps its pose at the moment of release
  - an overweight box makes a joint fall behind
- **Container check** (`scripts/ros_check.sh`, ~3 min per robot; irb1300 in the foreground, others in the background): a pick-and-place program passes, and `pose()` puts the box within 1 cm of the target.
- **Live acceptance:** the benchmark criterion in §1.

## 7. Out of scope: committed follow-ups

These are not in this spec, but they are planned. Each gets its own spec right after this one, in this order (user, 2026-10-17):

1. **Grippers:** real fingers gripping by contact replace the weld glue.
2. **Drag-in-3D workcell editor:** writes the same `workcell.yaml`.
3. **Run replay:** revives `2026-10-14-run-replay-design.md`, now recording the simulated arm and object poses.
4. **File ▸ Import:** URDF, MuJoCo zip, finished project, and workcell files.

After those come mobile bases, then RL, then real hardware and calibration.

## Task 1 findings (2026-10-17)

Measured on the irb1300 in the session image, `ros-jazzy-mujoco-ros2-control` 0.1.2 from apt, MuJoCo 3.16.0. All of the plan's gate checks passed.

- **(a) Hardware plugin:** `mujoco_ros2_control/MujocoSystemInterface`. Its node is the package's own `ros2 run mujoco_ros2_control ros2_control_node`, not controller_manager's. Services live under `/mujoco_ros2_control_node/`: `reset_world`, `set_free_joint_state`, `set_pause`, `step_simulation`.
- **(b) Pose publisher:** `mujoco_ros2_control_plugins/FreeJointStatePublisherPlugin`.
  - Parameters: `frame_id` ("" = world), `body_names` ([] = all free bodies), `topic` (default `free_joint_states`, namespaced under the node and plugin key), `publish_rate` (default 50 Hz).
- **(c) Message:** `mujoco_ros2_control_msgs/msg/FreeJointStateArray`. It has `free_joints[]`, and each entry has `name` (the body name) and `pose.pose` (`geometry_msgs/Pose`).
- **(d) Plugin loading:** plugins load from ROS parameters under `mujoco_plugins.<key>.type`, passed to the node as a parameters file, not as URDF hardware params. The plugin base class is `mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase`, and the MuJoCo header comes from `mujoco_vendor` (`/opt/ros/jazzy/opt/mujoco_vendor/include`).
- **(e) MuJoCo:** Python `mujoco` 3.16.0 with numpy 1.26.4 in the image, and 3.16.0 on the host.
- **Position actuators:** the exported `position` motors load as they are. The plugin matches them to joints and logs "Using MuJoCo position actuator". Controllers go active within 2 s.
- **(f) Restart time:** a sim-node restart plus controller spawn takes 2.6 to 3.3 s.
- **(g) MoveIt and reset:** MoveIt executes after a sim restart without restarting `move_group`. `reset_world` works while controllers stay active, and moves after a reset succeed.
- **(h) Topic reload:** not tested, because a restart takes only about 3 s.
- **Droop (new finding):** the exported motors are proportional only, so gravity leaves a steady error. joint2 settled at 0.321 rad for a 0.3 rad goal, and joint3 at 0.028 for a 0 rad goal. That is 0.016 to 0.028 rad, about 1 to 2 cm at the tool. A real arm's controller feeds gravity torque forward. Fix: route gravity compensation through the actuators (`gravcomp` on robot bodies, `actuatorgravcomp` on joints, `actuatorfrcrange` = the motor limit). The motors then supply the holding torque inside their torque limit, and a payload or a too-weak motor still droops.
