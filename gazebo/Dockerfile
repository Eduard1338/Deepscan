FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt update && apt install -y --no-install-recommends \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-plugins \   
    ros-humble-gazebo-ros2-control \
    ros-humble-web-video-server \
    ros-humble-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

COPY deepscan_ws /app/deepscan_ws
WORKDIR /app/deepscan_ws
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

COPY entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh
CMD ["/app/entrypoint.sh"]
