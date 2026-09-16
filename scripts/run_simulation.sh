#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /opt/drone_venv/bin/activate
source install/setup.bash

ros2 launch drone_gazebo simulation.launch.py
