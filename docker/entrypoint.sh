#!/usr/bin/env bash
set -e

# Continue with the container startup
exec gosu isaac-sim "$@"
source /opt/ros/humble/setup.bash
bash
