# Mobile Bases — Design

Date: 2026-10-19. Branch: `worktree-mobile-base`. This branch is for review and is not merged.

> **Status: built 2026-10-19.** Container check (fixture `mobi`): 26 PASS, 0 FAIL; drive_to accuracy and drive-pick-carry-place pass under MuJoCo. Changes from the text below: the true base pose comes from the glue plugin on `/sim/base_pose` (ros2_control refuses joints that aren't in the URDF, risk 1 in §10); `drive_to(..., reverse=True)` backs up; the path check sweeps the base outline through the real turn-drive-turn motion; a base without an arm is not in v1.
Builds on: MuJoCo programs (`2026-10-17-mujoco-programs-design.md`), the gripper (`2026-10-17-gripper-design.md`) and the workcell editor.

## 1. The picture

Today the robot is bolted to the floor: its base link *is* the world. A mobile base changes one thing. The base link sits on wheels on the floor, and the world becomes the floor itself, called `map`.

Everything else stays where it is. The arm still moves the way it does now, relative to its base. Objects still sit in the world. A program now has two kinds of move:
- **Drive** the base somewhere: `robot.drive_to(x, y, yaw)`.
- **Reach** with the arm, as before: `robot.move_to(...)`, `grip(...)`.

Both take world coordinates, so a box at (2, 0, 0.4) stays at (2, 0, 0.4) however the robot drives.

The physics must be the real thing, because RL will learn from it later:
- Wheels roll on the floor with friction and can slip.
- The base carries the arm's full weight.
- A heavy arm reaching too far tips the base.
- Wheel odometry drifts the way a real robot's does.

## 2. Scope

| In (v1) | Later |
|---|---|
| Differential drive: two driven wheels plus passive casters (TurtleBot-like) | Mecanum and omni wheels: rollers in MJCF plus ros2_controllers' `mecanum_drive_controller` (see §9) |
| Skid steer: two or more driven wheels per side (Husky-like), the same controller | Nav2 (maps, obstacle-avoiding paths) |
| One arm on the base, with or without the gripper | Whole-body planning (base and arm in one MoveIt plan) |
| A base with no arm (drive only) | Legged robots, steered (Ackermann) bases |

## 3. Decisions

| Topic | Decision | Why |
|---|---|---|
| **Base joint in MuJoCo** | Six single-axis joints: slide x, y, z, then hinge yaw, pitch, roll. A plain free joint is not used. | Arm gravity compensation must stay *inside* the robot. With a free joint, MuJoCo also pushes the arm's compensation force into the free joint, as an outside force lifting the robot. Measured: the floor carried 98 N of a 147 N robot. On single-axis joints that share can be clamped to zero (`actuatorgravcomp` plus `actuatorfrcrange ±1e-9`). The floor then carries 147.1 N, and a heavy arm reaching out tips the base. Gimbal lock comes only at 90° pitch, which means the robot has fallen over. MJX handles slide and hinge joints. |
| **Wheel contact** | A fitted cylinder per wheel (radius and width from the wheel's mesh, about its axis) with rubber friction (`1.0 0.005 0.0001`). The wheel mesh stays visual only. | A convex-decomposed wheel is a polygon, and it would ride bumpily. Cylinder–plane contact is native in MuJoCo and supported in MJX. |
| **Casters** | Found automatically: the base's lowest points, other than the wheels, within 5 mm of the wheel contact plane, clustered. Each becomes a frictionless (`condim="1"`) 1 cm sphere touching the floor. The base's own collision shapes stop touching the floor (contype bit 2) but still hit objects. | A real swivel caster rolls freely in every direction. A CAD caster is often just part of the base mesh, and as a hull it would drag on the floor. Ceiling: no swivel dynamics (caster flutter, side load). |
| **Wheel motors** | MuJoCo `velocity` actuators: `kv = effort / 0.5 rad/s`, force capped at the wheel's effort. The suggested effort is what accelerates the whole robot at 0.5 m/s² up a 5% slope, ×2. | This is what a real velocity-controlled wheel drive does. ros2_control's velocity command maps straight onto it. |
| **Forward** | +x of `base_link` (ROS REP-103), with the wheel axles along y. If the CAD's axles aren't along y, the Mobile base card offers "Turn the robot to drive along +x", which rotates every part and joint about z. | `diff_drive_controller` and every ROS tool assume it. Rotating the model once is simpler than carrying an offset frame everywhere. |
| **Floor height** | The base is spawned with its lowest wheel point on the floor (z = 0, plus 0.5 mm). The offset is written in the export. | CAD rarely puts the floor at z = 0. |
| **Controller (ROS)** | `diff_drive_controller` (Jazzy `ros-jazzy-ros2-controllers`, already in the image) named `base_controller`. Topic `~/cmd_vel` as `TwistStamped`. Wheel separation and radius come from the geometry. It publishes `odom → base_link`. | The standard tool. Odometry comes from wheel encoders, so it drifts when wheels slip, as on hardware. |
| **World frame and localization** | `map` is MuJoCo's world. The glue node `localization` publishes `map → odom` = truth × odom⁻¹, so `map → base_link` is the true pose. The truth comes from the base joints' states (MuJoCo, on `/joint_states`). | Perfect localization in sim. On hardware, AMCL or motion capture takes this node's place; nothing else changes. Programs, objects and MoveIt all live in `map`. |
| **MoveIt** | SRDF `virtual_joint type="floating" parent_frame="map" child_link="<base>"`. Floating, not planar, because the base sits above the floor and can pitch. The arm group is unchanged (a chain from the base link). | MoveIt reads the base pose from TF, so world objects stay put while the base drives. Base and arm are planned separately: MoveIt itself recommends not planning a base in MoveIt, and whole-body IK needs a custom plugin. |
| **Program API** | `drive_to(x, y, yaw=None, speed=0.3)`, `drive(v, w, seconds)`, `base_pose() -> (x, y, yaw)`. Arm calls take `map` coordinates and convert them to the base frame internally. | A small surface over the existing arm API. |
| **`drive_to`** | Turn to face the goal, drive straight, turn to the yaw. Closed loop on `map → base_link` at 20 Hz, finishing within 2 cm and 2°. Before moving, the straight path's footprint (the base's outline, plus 5 cm) is checked against the scene's objects. A blocked path raises `PathBlocked` and names the object. If the base stops making progress for 2 s it raises `MotionError` ("stuck"). | Honest and predictable. Ceiling: no path around obstacles (Nav2 is the upgrade). |

## 4. Build workbench

A **Mobile base** card in Connect, next to the Gripper card.
- **Wheel suggestions.** Joints from the base link to a round child link (its mesh within 3% of a cylinder about the joint axis) with a horizontal axis.
- **Ticks.** The user ticks the wheels. Left and right come from which side of the base a wheel is on.
- **Turn button.** "Turn the robot to drive along +x" appears when the axles aren't along y.
- **Shown values.** The card shows wheel radius, track width, drive type (2 wheels: differential; 4 or more: skid steer) and the casters found.
- **Warnings.** "Two wheels and no caster: the base will tip." "The wheels aren't the same size."
- **Model state.** `mobile: { left: jointId[], right: jointId[] } | null`, kept in the model so Undo covers it.
- **Wheel joints.** Ticked wheel joints become `continuous`. A wheel's `max_effort` is its motor strength (auto when empty).

## 5. Export

- **URDF.** Wheel joints are `continuous`, with an effort and a velocity limit of 20 rad/s. An extension tag, which URDF parsers skip like `<gripper>`:
  `<mobile_base drive="diff" left="wl_joint" right="wr_joint" wheel_radius="0.08" wheel_separation="0.48" ground="-0.12" casters="x y z r; ..."/>`
- **MJCF** (`build_mjcf` with `mobile`):
  - The base joints, with clamped gravcomp.
  - No gravcomp on the base and wheel bodies.
  - The wheel cylinders and caster spheres.
  - The base's own geoms set to contype or conaffinity 2.
  - Wheel velocity actuators.
  - The root body lifted to the floor.
- **Scene** (`scene.py`). Objects get `conaffinity="3"`, so they still touch the base's shapes. This is harmless for fixed robots.

## 6. Simulate workbench

- **3D view.** The view's world is `map` for a mobile robot. The live robot, replay and workcell are all drawn in it. The workcell's reach ring is hidden, because reach moves with the base.
- **Physics check.** Two checks are added:
  - **Holds still:** the base stays put for 2 s, with less than 5 mm of drift and no tilt.
  - **Drives straight:** the wheels at 0.2 m/s for 2 s move the base 0.4 m ± 10%, with heading drift under 3°.
- **Program brief.** The brief tells Claude about `drive_to`, `base_pose` and world coordinates, with a drive-then-pick example.

## 7. RL hooks (built later, not now)

- **Self-contained MJCF.** The exported MJCF needs no ROS: base joints, wheel velocity actuators and arm position actuators. A Gymnasium env can take its action as [left wheel velocity, right wheel velocity, arm targets…] and its observation from qpos/qvel plus object poses.
- **Same physics.** The env reuses `build_scene_xml` with the workcell, so RL trains on the same physics the programs run on.
- **MJX caveat.** MJX can't collide cylinders with boxes or meshes. A wheel bumping a box needs either a capsule stand-in or plain MuJoCo, and the RL work decides which.

## 8. Testing

- **Unit tests.** Wheel detection and cylinder fit. Caster finding. MJCF structure: base joints, contype, actuators, ground lift.
- **MuJoCo physics.** Floor force equals the whole robot's weight. Holds still. Drives straight. Turns in place by ±(v·t/separation). A heavy reach tips the base.
- **Chain.** A mobile robot parses: wheels are not arm joints, and the `mobile_base` tag is read.
- **MoveIt config.** The virtual joint, `base_controller`, and wheel command interfaces.
- **Robot API.** `drive_to` math with fake TF: the turn–drive–turn order, `PathBlocked`, stuck detection.
- **Container check** on a fixture mobile manipulator (`mobi`: a 0.5 m chassis, two wheels, a caster, the six-axis arm with gripper):
  - controllers active (with `base_controller`);
  - the `map → odom → base_link` TF chain exists;
  - `drive_to(1, 0.5, 90°)` arrives within 2 cm;
  - a physics program drives to a table and picks and places a box there.
- **Frontend.** The card's suggestions, ticks, warnings and turn button; the model reducer; the export request.

## 9. Mecanum, for later

ros2_controllers ships `mecanum_drive_controller`. In MJCF, each wheel is a hub with 8–12 rollers on free hinge joints at ±45°, and the roller collision shapes are capsules (Isaac Sim's TIAGo model and Chrono's demo do the same). Detecting mecanum wheels from CAD is hard, so the user would tick "mecanum" on the card. It is a separate spec once v1 is reviewed.

## 10. Risks

- **State-only base joints.** `mujoco_ros2_control` might refuse the six base joints as state-only joints, because they aren't in the URDF. Fallback: a small glue node reads MuJoCo's truth from the base body pose instead, via a free-joint-style plugin or `/sim` service.
- **TF tree.** MoveIt's floating virtual joint plus diff-drive TF plus `map → odom` must form one tree: map → odom → base_link → …. `robot_state_publisher` must not also publish a root for the base.
- **Odometry reset.** `reset_world` puts MuJoCo back, but the controller's odometry carries on. The localization node absorbs that, since `map → odom` changes; programs only use `map`.
