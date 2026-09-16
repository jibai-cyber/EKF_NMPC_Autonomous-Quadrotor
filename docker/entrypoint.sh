#!/usr/bin/env bash
set -e

mkdir -p "${HOME}"
mkdir -p "${XDG_RUNTIME_DIR:-/tmp/drone_runtime}"
chmod 700 "${XDG_RUNTIME_DIR:-/tmp/drone_runtime}"
source /opt/ros/jazzy/setup.bash
source /opt/drone_venv/bin/activate

if [[ -f /workspace/install/setup.bash ]]; then
  source /workspace/install/setup.bash
fi

exec "$@"
