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
exit "$status"
