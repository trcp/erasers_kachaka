#!/usr/bin/bash

source /home/roboworks/colcon_ws/install/setup.bash

ros2 launch erasers_kachaka_bringup bringup.launch.py bringup_type:=${TYPE_BRINGUP} shelf_type:=${TYPE_SHELF}
