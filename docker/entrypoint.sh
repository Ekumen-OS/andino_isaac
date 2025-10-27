#!/bin/bash
set -e

# Continue with the container startup
exec gosu isaac-sim /isaac-sim/python.sh "$ISAAC_LAUNCH_SCRIPT"
