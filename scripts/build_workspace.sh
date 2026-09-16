#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /opt/drone_venv/bin/activate

if [[ ! -f "${HOME}/.ros/rosdep/sources.cache/index" ]]; then
  rosdep update
fi

# Jazzy's current rosdep index has no system mapping for the ament_python build
# type; python3-colcon-ros in the image provides that build extension.
rosdep install \
  --from-paths src \
  --ignore-src \
  --skip-keys ament_python \
  -r -y \
  --rosdistro jazzy
python3 -m colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
