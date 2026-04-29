#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/humble/setup.bash


# Build the workspace if the package isn't built yet
if [ -d "/workspace/src" ]; then
  echo "Building ROS2 workspace..."
  cd /workspace
  if ! colcon build --symlink-install; then
    echo "[entrypoint] Build failed. Staying alive for inspection..."
    exec bash -lc 'trap : TERM INT; sleep infinity & wait'
  fi
  echo "Workspace built successfully!"
fi

# Source the workspace
source /workspace/install/setup.bash

# Execute the command passed to the entrypoint
exec "$@"
