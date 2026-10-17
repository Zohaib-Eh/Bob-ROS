# Gripper: Design

**Date:** 2026-10-17
**Builds on:** `2026-10-17-mujoco-programs-design.md` (MuJoCo part C: physical arm and objects, `workcell.yaml` + `SCENE`, the glue plugin), and the Build steps (Parts → Group → Connect → Export).
**Roadmap:** item 3b. After this come the drag-in-3D workcell editor, run replay, and File ▸ Import.

## 1. Intent

Today a program picks up a box with `attach()`, and in physics mode the box is glued to the flange by a weld. The user wants the real thing: a real gripper from GrabCAD on the irb1300 that holds boxes by squeezing them, so the simulation behaves as it would on the shop floor.

The user decided the flow (2026-10-17):
- They download a gripper STEP from GrabCAD.
- In **Build**, they add it to the existing robot project.
- They mount it on the flange.
- They give it finger joints with the Build tools they already use.
- The gripper then works in Simulate and in Program mode.

Success looks like this:
- In Build, a GrabCAD two-finger gripper is imported into the irb1300 project, mounted flush and centred on link6's flange, and its fingers get sliding joints marked as the gripper.
- In Program mode under MuJoCo, a program opens the gripper, closes it on a 0.2 kg box, lifts it, carries it, opens, and the box lands within 1 cm of the target. The box is held by contact and friction only: no weld, no glue.
- A box too heavy for the grip force slips out. The program gets a plain message saying so, instead of the run silently going on.
- Robots without a gripper behave exactly as today, with the glue `attach()`.

## 2. Goals, non-goals, anti-goals

**Goals:**
- Add a gripper from a second CAD file and mount it onto the flange in Build.
- Parallel-jaw grippers: two fingers that slide along a straight line, the second mirroring the first. One-finger grippers (a finger against a fixed jaw) are out of this round (my call).
- Real grasping in MuJoCo: force-limited finger motors, contact, friction, slip.
- A standard ros2_control gripper controller and a MoveIt gripper group, so the stack matches a real cell.
- The program API `grip()` / `release()` and gripper state, with Claude's brief updated.
- The tool point (TCP) between the fingertips, so `move_to` positions the grasp point, not the flange.

**Non-goals for this round** (my call; the user can pull any of them in):
- Pivoting-finger grippers (fingers on a hinge or a linkage, like a Robotiq 2F-85), suction cups, and multi-finger hands.
- Tool changers, or more than one gripper per robot.
- Grasp planning, meaning the app choosing where to grab. Programs still say where.

**Anti-goals** (would count as failure even if it technically works):
- Faking the grip, by welding the box when the fingers close. That is what we are replacing.
- Breaking robots without a gripper, or forcing every robot to have one.
- A mount step so fiddly that getting the gripper flush on the flange takes more than a few clicks.

## 3. Constraints and facts from the code

- **Import already adds parts.** File ▸ Import CAD… adds the new file's bodies to the open project (reducer `import` appends). Parts keep their CAD coordinates, so a GrabCAD gripper lands wherever its file put it, usually near the origin.
- **No tool moves parts today.** Body geometry (vertices, face centres, normals, axes) lives in the frontend model and is sent to the backend on export.
- **The arm can't branch today.** `app/ros/chain.py` `parse_chain` rejects more than one moving leaf joint ("branching is declined"). Two fingers make a branch, so gripper joints must be kept out of the arm chain.
- **Joints already carry what the gripper needs:** type (prismatic supported), limits, and `max_effort` (Max torque, which is force in N for sliding joints).
- **Physics stack (from part C):**
  - The plugin is `mujoco_ros2_control/MujocoSystemInterface`, and motors are MuJoCo `position` actuators with a force range.
  - Gravity is fed through the motors (`actuatorgravcomp`).
  - The sim restarts per scene change and is reset with the arm controller off.
  - The plugin docs recommend a tendon actuator plus an equality constraint for parallel grippers.
- **Fingers must be separate bodies.** GrabCAD models sometimes merge fingers into one solid. The app must say so plainly; it can't split solids.
- Units are metres, kilograms and newtons. Everything shown to the user is in plain language.

## 4. What the user decided

| Topic | Decision (the user's) |
|---|---|
| Gripper source | A real gripper downloaded from GrabCAD as STEP |
| Where it's added | The Build workbench, into the existing robot project |
| How it's attached | Mounted on the flange in Build; it becomes part of the robot |
| Fidelity | Real-world behaviour (standing preference from part C): the grip is by contact and friction |
| Order | Gripper before the 5-task benchmark |

## 5. The design

### 5.1 Build: add and mount

- **Add part:** the File menu gets **Add part from CAD…**. It works like Import CAD… (appending) and is the obvious entry once a project exists. Each imported body records which file it came from (`source`), so the mount tool can move "everything from gripper.step" as one piece.
- **Mount tool** (Parts step, available when the project has bodies from more than one file):
  1. Pick a flat face on the part being mounted (the gripper's base or mounting plate).
  2. Pick a flat face on the robot (link6's flange face).
  3. The app moves all bodies from that file rigidly so the faces meet: centres coincide (circle centres for round faces, as joint detection already uses), normals point at each other, and there is no gap.
  4. A **Turn** control rotates the mounted part about the shared axis, in 15° steps with a free-angle field, and a **Flip** button turns it 180° about an axis in the face.
  5. **Undo** puts it back.
- The transform is applied to the stored body geometry: vertices, face centres, normals, axes. It is saved with the project and also changes the exported meshes.

### 5.2 Build: gripper joints

- In Connect, the gripper's base is joined to link6 with a **fixed** joint, using the existing tools.
- Each finger is joined to the base with a **sliding (prismatic)** joint, using existing detection and the manual joint tool. Its limits are the finger's travel, and Max torque (N) is the grip force per finger.
- A new **Gripper** card in Connect:
  - "These joints are the gripper's fingers": tick the two prismatic joints.
  - The second **mirrors** the first: it moves the same distance in the opposite direction.
  - It shows open width, closed width and grip force in plain words.
  - It checks:
    - Fingers that are one solid: "The fingers are one part; the gripper needs separate finger parts. Pick a different model or split it in your CAD tool."
    - Finger axes not parallel: "The fingers don't slide along the same line."
    - Missing limits.
- **Tool point (TCP):** set automatically to the point midway between the fingertips, on the closing line, at the fingertips' far end. The user can nudge it along the gripper axis with a number field. TCP is shown as a marker in the viewer.

### 5.3 Export

- The URDF gains the gripper links and joints as usual. The second finger gets a URDF `<mimic>` of the first, with multiplier 1 and the joints' axes set so that the same value closes both. A fixed `tcp` link is added at the tool point.
- `<robot>/config/gripper.yaml` in the description package records the finger joints, mimic, open and closed positions, grip force and the tcp link. Everything downstream reads gripper facts from here.
- `parse_chain` reads `gripper.yaml` when present. It excludes the finger joints from the arm, and the arm's tip becomes `tcp`. Robots without the file parse exactly as today.
- MJCF:
  - The finger joints get position actuators force-limited to the grip force.
  - The mirrored finger is coupled with a joint `<equality>` (per the plugin's parallel-gripper guidance; the builder may use a tendon plus equality if that is what the plugin needs).
  - Finger pads use friction 1.0 by default.
  - **Collision shapes (my call):** MuJoCo collides a mesh as its convex hull, so L-shaped GrabCAD fingers, or a housing wrapped around the jaws, would stall on the hull before reaching the box. At export, the gripper's links (base and fingers) are split into convex pieces with **CoACD** (`coacd` from pip), and each piece is its own collision geom on the link. Arm links stay single meshes. Visual meshes are unchanged. The test fixture's fingers are L-shaped, so this is actually tested.
  - The motor-sizing and hold-check logic leave the finger joints out of the arm's numbers.

### 5.4 ROS and MoveIt

- **ros2_control:** the finger joint (the leader) is in the hardware block. The mirrored one is reported through the mimic/equality.
- **Controller:** a gripper controller from ros2_controllers (`parallel_gripper_action_controller`) with its standard GripperCommand-style action. Max effort is the grip force.
- **MoveIt:**
  - SRDF gets a `gripper` group, `end_effector` on `tcp`, and open and closed named states.
  - The arm group's tip is `tcp`.
  - `moveit_controllers.yaml` lists the gripper controller.
  - Collisions between the fingers and the gripper base are disabled. Collisions between the fingers and an object are allowed while it is being gripped (MoveIt attach with the finger links as touch links).
- **Task 1 of the plan** checks that the gripper controller package is in the Jazzy image and works with the MuJoCo plugin, and that mimic and equality coupling works there. If not, stop and come back, as part C did.

### 5.5 Programs (`robot.py`)

- **`r.release()`:** opens the gripper fully and waits until it has stopped. If something was held, the MoveIt attachment is removed and the object falls or stays where it is.
- **`r.grip(name=None)`:**
  - Closes until the fingers stop: stalled on an object, or fully closed.
  - If they stalled with a gap left, the object is held. With `name`, it is attached in MoveIt so planning carries it. Without `name`, the held object is the one the fingers touch.
  - If they closed fully: `GripError("The gripper closed on nothing ...")`.
- **`r.gripper_width()`:** the current opening, in metres.
- **Jaw direction (my call):** `r.tool_down(yaw=0.0)` gains a `yaw` (radians, about the vertical). It turns the tool about its approach axis so the jaws close along base-frame direction `(cos yaw, sin yaw, 0)`. `tool_facing` gains the same optional `yaw`. The brief says to set yaw so the jaws close across two opposite faces of the box, meaning the box's own yaw, or that plus 90°.
- **Slip watch:** while something is held, every motion checks the object's pose relative to the TCP. If it drifts more than 1 cm, the program gets `GripError("'box' slipped out of the gripper while moving (0.2 kg, grip force 40 N per finger) ...")`.
- `attach` / `detach` stay. On a robot with a gripper they call `grip(name)` / `release()`. On a robot without one, they are the part C glue.
- **Mock hardware:** the fingers jump to their target, `grip(name)` attaches in MoveIt (as `attach` does today), and there is no slip.
- **Brief:**
  - the gripper's open width and grip force, and that `move_to` targets the TCP
  - how to grasp: open, approach from above, descend so the box sits between the pads, grip, lift
  - that heavy or smooth objects can slip

### 5.6 Simulate workbench

- The Physics card's hold check includes the gripper. The fingers hold their open position, and the arm's holding check uses the gripper's mass.
- The 3D view shows the fingers moving, through `/tf` from the mimicked joint states.

## 6. Errors the user can see

| Situation | Message (plain words) |
|---|---|
| Fingers are one solid | Gripper card: "The fingers are one part; the gripper needs separate finger parts…" |
| Finger axes not parallel | Gripper card: "The fingers don't slide along the same line." |
| Mount faces not flat | Mount tool: "Pick a flat face (the gripper's mounting plate and the flange)." |
| Grip closed fully | `GripError`: "The gripper closed on nothing: no object between the fingers at (x, y, z)." |
| Object slipped | `GripError`: "'box' slipped out of the gripper while moving (mass, grip force)…" |
| Object wider than the open width | `GripError` from `grip`: "'box' is 0.12 m wide; the gripper opens to 0.08 m." This check needs a named object. |
| Gripper controller missing in the image | Not shipped: plan Task 1 checks the controller works with the MuJoCo plugin and stops if not. There is no runtime fallback, because gluing a gripper robot is the faking this replaces (my call). |

## 7. Testing

- **Fixture:** a generated two-finger gripper STEP (OCC primitives) in `tests/conftest.py`: a base plate and two separate **L-shaped** finger bodies, with the pad curling inward, so that convex-hull collision would fail. No GrabCAD file is needed for tests.
- **Backend (decomposition):** the gripper links' collision geoms are several convex pieces. A box between the L-fingers is reached and gripped, which a single hull would block.
- **Frontend:**
  - the mount transform on a cube meeting a cube face, including turn and flip
  - project save and load keeps the moved geometry
  - Gripper card validation (one solid, non-parallel axes, missing limits)
- **Backend:**
  - `gripper.yaml` export
  - `parse_chain` with and without a gripper (the tip becomes `tcp`; fingers are excluded)
  - MJCF finger actuators and coupling (MuJoCo Python: both fingers close together, and the force limit holds)
  - an MJCF grasp test: close on a box, lift it, it stays; a box over the friction limit slips
- **Container** (`scripts/ros_check.sh`, irb1300 plus the generated gripper, in the foreground, ~5 min): a physics program that releases, grips the box, carries it, releases it, and checks `pose()` within 1 cm. A second program with a heavy box expects `GripError` (slip).
- **Live:** the user imports a real GrabCAD gripper and runs pick-and-place in Program mode.

## 8. Left to the builder

- The exact UI layout of the mount tool and the Gripper card, following the existing card and step styles.
- Whether the mirrored finger uses a joint equality or a tendon plus equality, whichever `mujoco_ros2_control` drives correctly (decided in plan Task 1).
- The TCP fingertip-end rule for odd finger shapes (use the finger bodies' extent along the gripper axis).
- How slip is sampled. After each motion is the minimum, and during motion is better if cheap.

## Task 1 findings (2026-10-17)

The probe was a two-finger mimic gripper under `mujoco_ros2_control`, closing on a 0.04 m box on a pedestal. All gate checks passed:

- **Controller:** `parallel_gripper_action_controller/GripperActionController`, from `ros-jazzy-parallel-gripper-controller` 4.42.1 (already pulled in by ros2-controllers).
  - Action: `control_msgs/action/ParallelGripperCommand` on `/gripper_controller/gripper_cmd`. The command is a `sensor_msgs/JointState` (name, position), and the result has `state`, `stalled` and `reached_goal`.
  - Parameters: `joint`, `max_effort`, `allow_stalling`, `stall_timeout`, `stall_velocity_threshold`, `goal_tolerance`, `state_interfaces`, `max_effort_interface`, `max_velocity`.
- **Mimic:** declared in the ros2_control block as `<joint name="follower"><param name="mimic">leader</param><param name="multiplier">1</param> state interfaces only</joint>`, plus the URDF `<mimic>`. `mujoco_ros2_control` logs "is a mimic joint mimicking joint …". Its "Unable to find the actuator" warning for the follower is expected. `/joint_states` reports both fingers.
- **MuJoCo:** the URDF importer turns `<mimic>` into a joint equality by itself (`neq = 1`, type joint), so the export adds no extra coupling.
- **Contact:** the fingers stop on the box.
  - *The coupling is soft under load.* The leader stopped at 15.2 mm and the follower at 7.3 mm, so the equality gives way under grip force. The export must stiffen it (equality `solref`), and Task 6 tests the fingers stay within 1 mm while squeezing.
  - *The controller reported `reached_goal` and not `stalled`*, because the remaining error was inside its default `goal_tolerance`. Set `goal_tolerance: 0.001`. `robot.py` judges the grip by finger width (`grip_logic.outcome`), not by these flags.
- **CoACD:** `coacd` 1.0.14 installs on the host and in the image (numpy 1.26.4 kept).
