#!/usr/bin/env bash
# Session container entry: build the workspace, run MoveIt on mock hardware and rosbridge.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd /ws && colcon build 2>&1 | tail -n 5
source /ws/install/setup.bash
ROBOT=$(basename "$(ls -d src/*_moveit_config)" _moveit_config)
ros2 launch "${ROBOT}_moveit_config" demo.launch.py &
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 &
until ros2 action list 2>/dev/null | grep -q '^/move_action$'; do sleep 1; done
touch /tmp/ready
echo "session ready: ${ROBOT}"
wait
