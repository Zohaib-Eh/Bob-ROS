# ROS 2 Jazzy with MoveIt, TRAC-IK, ros2_control, MuJoCo (mujoco_ros2_control) and EAIK:
# builds and checks generated workspaces, and runs the Program-mode session.
FROM ros:jazzy
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-moveit \
        ros-jazzy-trac-ik-kinematics-plugin \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-xacro \
        ros-jazzy-rosbridge-suite \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*
# Keep the Debian numpy that ROS was built against; pip would otherwise replace it with 2.x.
RUN python3 -c "import numpy; print('numpy==' + numpy.__version__)" > /tmp/pin.txt \
    && pip3 install --no-cache-dir --break-system-packages -c /tmp/pin.txt EAIK
# Separate layers so the MoveIt layer above stays cached.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-mujoco-ros2-control \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir --break-system-packages -c /tmp/pin.txt mujoco
# Gripper: the parallel-gripper controller, and CoACD for convex finger collision pieces.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-parallel-gripper-controller \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir --break-system-packages -c /tmp/pin.txt coacd
