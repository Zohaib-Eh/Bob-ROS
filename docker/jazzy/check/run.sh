#!/usr/bin/env bash
# Runs inside the cad-to-urdf-jazzy container with the generated workspace at /ws.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd /ws
colcon build 2>&1 | tail -n 20
source install/setup.bash
ROBOT=$(basename "$(ls -d src/*_moveit_config)" _moveit_config)
echo "PASS colcon build: ${ROBOT}"
ros2 launch "${ROBOT}_moveit_config" demo.launch.py > /tmp/launch.log 2>&1 &
LAUNCH=$!
trap 'kill $LAUNCH 2>/dev/null || true; wait $LAUNCH 2>/dev/null || true' EXIT
status=0
python3 /check/check_ros.py "$ROBOT" || status=1
python3 /check/check_ik.py "$ROBOT" || status=1
for p in /check/programs/*.py; do
  out=$(cd /ws/src/${ROBOT}_tasks && PYTHONPATH=/ws/src/${ROBOT}_program:$PYTHONPATH timeout 180 python3 -u -m runner "$p" 2>&1) || true
  if echo "$out" | grep -q "^RESULT: ok"; then echo "PASS robot.py $(basename "$p")"
  else echo "FAIL robot.py $(basename "$p")"; echo "$out" | tail -n 15; status=1; fi
done
if [ "$status" -ne 0 ]; then
  echo "--- last launch log lines ---"
  tail -n 60 /tmp/launch.log
fi

# Physics phase: the same session start as the app (sim_ctl boot, MoveIt, scene_sync), then this
# robot's programs in /check/physics, each run twice (the second run takes the reset_world path).
shopt -s nullglob
physics=(/check/physics/${ROBOT}_*.py)
if [ ${#physics[@]} -gt 0 ]; then
  kill $LAUNCH 2>/dev/null || true; wait $LAUNCH 2>/dev/null || true
  export PYTHONPATH=/ws/src/${ROBOT}_program:$PYTHONPATH
  # a robot's own workcell (workcell.yaml + meshes/), e.g. CAD fixtures imported by the app
  if [ -d "/check/physics/${ROBOT}_workcell" ]; then cp -r "/check/physics/${ROBOT}_workcell/." "/ws/src/${ROBOT}_tasks/"; fi
  python3 -m sim_ctl hardware
  ros2 launch "${ROBOT}_moveit_config" demo.launch.py > /tmp/launch_physics.log 2>&1 &
  LAUNCH=$!
  until ros2 action list 2>/dev/null | grep -q '^/move_action$'; do sleep 1; done
  python3 -m sim_ctl boot
  if [ "$(cat /tmp/physics 2>/dev/null)" != mujoco ]; then
    echo "FAIL physics boot: $(cat /tmp/physics 2>/dev/null)"; tail -n 30 /tmp/sim/control.log 2>/dev/null; exit 1
  fi
  echo "PASS physics boot"
  python3 -m scene_sync > /tmp/scene_sync.log 2>&1 &
  for p in "${physics[@]}"; do
    for n in 1 2; do
      out=$(cd /ws/src/${ROBOT}_tasks && timeout 300 python3 -u -m runner "$p" 2>&1) || true
      if echo "$out" | grep -q "^RESULT: ok"; then echo "PASS physics $(basename "$p") run $n: $(echo "$out" | grep ' ok' | tail -n 1)"
      else echo "FAIL physics $(basename "$p") run $n"; echo "$out" | tail -n 15; status=1; fi
    done
  done
  if [ "$status" -ne 0 ]; then
    echo "--- sim log ---"; tail -n 30 /tmp/sim/control.log
    echo "--- launch log ---"; tail -n 30 /tmp/launch_physics.log
  fi
fi
exit "$status"
