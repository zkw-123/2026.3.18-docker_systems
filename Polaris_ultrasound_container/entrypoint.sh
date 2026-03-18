#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/humble/setup.bash


# Build the workspace if the package isn't built yet
if [ -d "/ros2_ws/src" ] && [ ! -d "/ros2_ws/install/polaris_ultrasound" ]; then
  echo "Building ROS2 workspace..."
  cd /ros2_ws
  if ! colcon build --symlink-install; then
    echo "[entrypoint] Build failed. Staying alive for inspection..."
    exec bash -lc 'trap : TERM INT; sleep infinity & wait'
  fi
  echo "Workspace built successfully!"
fi

# Source the workspace
source /ros2_ws/install/setup.bash

# Execute the command passed to the entrypoint
exec "$@"
