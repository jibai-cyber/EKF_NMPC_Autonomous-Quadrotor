#!/usr/bin/env bash
set -euo pipefail

if ! command -v docker >/dev/null 2>&1; then
  echo "Docker is required because ROS 2 Jazzy targets Ubuntu 24.04, while this host is newer."
  sudo apt-get update
  sudo apt-get install -y docker.io docker-compose-v2
fi

if ! groups | grep -qw docker; then
  current_user="$(id -un)"
  echo "Adding ${current_user} to the docker group."
  sudo usermod -aG docker "${current_user}"
  echo "Docker group membership has been configured."
  echo "Log out and back in, then run this script again to build the image."
  exit 0
fi

docker compose build
