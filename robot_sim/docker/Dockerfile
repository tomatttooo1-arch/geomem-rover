# Build context: repository root. Build --target agx_ros on AGX (native arm64),
# --target simulator on an amd64 server. No Jetson driver or CUDA installation.
ARG ROS_BASE_IMAGE=public.ecr.aws/docker/library/ros:humble-ros-base-jammy
FROM ${ROS_BASE_IMAGE} AS agx_ros
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions python3-yaml \
    ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-tf2-ros \
    ros-humble-cv-bridge ros-humble-message-filters \
    && rm -rf /var/lib/apt/lists/*
COPY robot_sim/fpga_sim /opt/fpga_ws/src/fpga_sim
WORKDIR /opt/fpga_ws
RUN source /opt/ros/humble/setup.bash && colcon build --merge-install
COPY robot_sim/docker/entrypoint.sh /opt/fpga_entrypoint.sh
ENTRYPOINT ["/bin/bash", "/opt/fpga_entrypoint.sh"]
CMD ["ros2", "launch", "fpga_sim", "navigation.launch.py"]

FROM agx_ros AS simulator
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-gz ros-humble-rviz2 libgl1-mesa-dri xvfb xauth \
    && rm -rf /var/lib/apt/lists/*
# Software-rendered starter avoids claiming shared GPU resources. Xvfb supplies
# a private display; do not mount the host's X11 socket or run privileged.
ENV LIBGL_ALWAYS_SOFTWARE=1
CMD ["xvfb-run", "-a", "-s", "-screen 0 1280x720x24", "ros2", "launch", "fpga_sim", "simulation.launch.py"]
