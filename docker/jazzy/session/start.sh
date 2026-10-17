#!/usr/bin/env bash
# Session container entry: build the workspace, boot the MuJoCo sim (or mock hardware), run MoveIt and rosbridge.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
echo build > /tmp/stage
cd /ws && colcon build 2>&1 | tail -n 30
source /ws/install/setup.bash
ROBOT=$(basename "$(ls -d src/*_moveit_config)" _moveit_config)
echo physics > /tmp/stage
export PYTHONPATH=/ws/src/${ROBOT}_program:$PYTHONPATH
# MuJoCo when the export has a model (writes /tmp/sim/hardware for the launch, /tmp/physics for the app)
python3 -m sim_ctl boot || { mkdir -p /tmp/sim; echo mock > /tmp/sim/hardware; echo "off: sim boot crashed" > /tmp/physics; }
echo moveit > /tmp/stage
ros2 launch "${ROBOT}_moveit_config" demo.launch.py &
LAUNCH=$!
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 &
BRIDGE=$!
DEADLINE=$((SECONDS + 180))
until MOVEIT=$(ros2 action list 2>/dev/null | grep -c '^/move_action$'); (( MOVEIT )) && bash -c '</dev/tcp/127.0.0.1/9090' 2>/dev/null; do
  (( MOVEIT )) && echo bridge > /tmp/stage  # MoveIt up, bridge pending
  kill -0 "$LAUNCH" 2>/dev/null || { echo "MoveIt stopped during startup"; exit 1; }
  kill -0 "$BRIDGE" 2>/dev/null || { echo "rosbridge stopped during startup"; exit 1; }
  (( SECONDS < DEADLINE )) || { echo "session startup timed out after 180 s"; exit 1; }
  sleep 1
done
touch /tmp/ready
echo ready > /tmp/stage
echo "session ready: ${ROBOT}"
wait -n
exit 1
