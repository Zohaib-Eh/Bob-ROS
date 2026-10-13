# ROS 2 Jazzy with MoveIt, TRAC-IK, ros2_control and EAIK: builds and checks generated workspaces.
FROM ros:jazzy
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-moveit \
        ros-jazzy-trac-ik-kinematics-plugin \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-xacro \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*
# Keep the Debian numpy that ROS was built against; pip would otherwise replace it with 2.x.
RUN python3 -c "import numpy; print('numpy==' + numpy.__version__)" > /tmp/pin.txt \
    && pip3 install --no-cache-dir --break-system-packages -c /tmp/pin.txt EAIK
