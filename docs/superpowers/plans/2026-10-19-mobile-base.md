# Mobile Bases — Plan

**Goal.** Differential-drive and skid-steer bases, with or without an arm, in Build, the export, MuJoCo, ROS and programs. They have real wheel physics, which RL will reuse later.

**Spec:** `docs/superpowers/specs/2026-10-19-mobile-base-design.md`

**Branch:** `worktree-mobile-base`, for review only and not merged.

Every task is test-first and committed when green.

1. **Wheel geometry** (`backend/app/export/mobile.py`).
   - `fit_wheel(vertices, axis, center)` returns radius, width and centre, and says whether the wheel is round (within 3%).
   - `find_casters(base_vertices, wheel_planes)` returns spheres.
   - `drive_info(links, joints, bodies, mobile)` returns drive type, left/right joint names, radius, separation, ground and casters.
   - A `Mobile` model goes into `ExportRequest`.
   - Tests use a box chassis, cylinder wheels and a caster bump.
2. **URDF tag.** `export_urdf(..., mobile=)` writes the wheels as continuous joints with limits, plus the `<mobile_base .../>` tag. `export_package` passes `mobile` through, and the export route takes it.
3. **MJCF** (`build_mjcf(..., mobile=)`).
   - The six base joints with clamped gravcomp; no gravcomp on the base or wheels.
   - Wheel cylinders and caster spheres; base geoms on contype/conaffinity 2.
   - Wheel velocity actuators; the root lifted to the floor.
   - Physics tests in MuJoCo: floor force, holds still, drives straight, turns in place, a heavy reach tips.
4. **Scene.** Objects get `conaffinity 3`. Check: an object resting on the base doesn't fall through it.
5. **Chain.** `parse_chain` reads `<mobile_base>`; wheels are not arm joints; `Chain.mobile`. A base with no arm is allowed when it is mobile.
6. **MoveIt and ros2_control.**
   - The SRDF floating virtual joint `map → base`.
   - The xacro: wheels with velocity commands; base joints with state interfaces only.
   - `base_controller` (diff_drive) in `ros2_controllers.yaml`, the launch spawner and `sim_ctl.controller_names`.
   - The `diff_drive_controller` package dependency.
7. **Localization glue node.** It reads `/joint_states` base_* and TF `odom → base`, and publishes `map → odom`. It is started by the launch and by `sim_ctl`, mobile only.
8. **robot.py.**
   - `self.world` is `map` when mobile.
   - Objects and user coordinates are in the world, converted to the base frame for the arm math.
   - `drive_to`, `drive` and `base_pose`, plus the `PathBlocked` error.
   - Unit tests with a fake TF and a fake cmd_vel.
9. **Brief, example and physics check.**
   - The program brief's mobile section and a `drive_pick` example.
   - The app's physics check adds "Holds still" and "Drives straight" for mobile robots.
10. **Frontend.**
    - `ModelState.mobile`; reducer actions `setMobile` and `turnRobot`.
    - `model/mobile.ts` suggests wheels (round about the axis, horizontal axis, on the base) and warns.
    - The `MobileCard` in Connect; the export request carries `mobile`.
    - The Simulate world frame is `map`; the reach ring is hidden.
11. **Fixture robot and container check.**
    - `tests/ros_helpers.mobile_arm()` builds `mobi`.
    - Container physics programs: `mobi_drive.py` (drive_to accuracy) and `mobi_drive_pick.py`.
    - `run.sh` checks for `base_controller` and the TF chain.
12. **Browser pass.** Build a mobile robot in the UI (the fixture, saved as a project), tick the wheels, Simulate, run the drive-pick program, replay it.
