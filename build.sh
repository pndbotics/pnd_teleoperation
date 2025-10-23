#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
colcon build --packages-skip pteleop_bridge tests_bag