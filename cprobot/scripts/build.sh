#!/usr/bin/env bash
set -e

colcon build --symlink-install

echo ""
echo "To use the build:  source $(pwd)/install/setup.bash"
