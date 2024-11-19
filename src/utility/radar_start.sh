#!/bin/bash
export ROS_DISTRO=foxy
source /opt/ros/foxy/setup.bash
source ~/.radar_ros2_ws/install/setup.bash
source ~/.radar_ros2_ws/install/local_setup.bash
ros2 launch umrr_ros2_driver radar.launch.py
