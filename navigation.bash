#!/bin/bash

echo "Map Path: ${MAP_PATTH}"


. /home/roboworks/colcon_ws/install/setup.bash

ros2 launch erasers_kachaka_navigation navigation_launch.py map:=${MAP_PATH}
