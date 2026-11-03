FROM px4io/px4-sitl-gazebo-ros2@sha256:2c01c89215a5f541d3a819c1bd2115ff27c9423679f3738dbb4096271d77084b

ENV DEBIAN_FRONTEND=noninteractive
COPY packages.lock.json install_locked.py bridge.yaml /opt/drone_cv/g1/
RUN python3 /opt/drone_cv/g1/install_locked.py

# ROS vendor hooks otherwise hide native sim8.yaml from the vendor gz CLI.
# Seed before the inherited ROS entrypoint sources its environment hooks.
ENV GZ_CONFIG_PATH=/usr/share/gz

# All ROS nodes use Gazebo /clock; OS-time synchronization is wrong at low RTF.
# Supported PX4 startup env override, scoped to this simulation image.
ENV PX4_PARAM_UXRCE_DDS_SYNCT=0

# Preserve the parent's ROS/PX4 entrypoint; no simulation or control starts here.
LABEL drone-cv.component="g1-ros-camera-bridge"
