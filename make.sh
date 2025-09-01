#!/bin/bash
set -e
cd "$(dirname "$0")"
colcon build --packages-select polygon_stamped_sub_ros
source install/setup.bash
ros2 run polygon_stamped_sub_ros polygon_stamped_sub
