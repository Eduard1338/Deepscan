#!/bin/bash
source /opt/ros/humble/setup.bash
source /app/deepscan_ws/install/setup.bash
exec ros2 launch deepscan_gazebo bringup.launch.py
