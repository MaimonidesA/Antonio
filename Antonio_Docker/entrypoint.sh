#!/bin/bash

set -e

source /opt/ros/humble/setup.bash

cd /home/antonio/antonio_ws/

colcon build --symlink-install 

source /home/antonio/antonio_ws/install/setup.bash

ros2 launch antonio_description display.launch.py

echo "Provided arguments: $@"

exec $@
