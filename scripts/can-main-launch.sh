#!/bin/bash
source "$(pwd)/install/local_setup.sh"
"$(pwd)/scripts/can-setup.sh"
ros2 launch "$(pwd)/launch/main_launch.py"