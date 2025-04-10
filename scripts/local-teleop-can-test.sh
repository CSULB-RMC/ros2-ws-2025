#!/bin/bash
source "$(pwd)/install/local_setup.sh"
ros2 launch $(pwd)/launch/teleop_launch.py > /dev/null 2>&1 &
$(pwd)/scripts/launch.sh