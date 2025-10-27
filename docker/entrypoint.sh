#!/bin/bash
set -e

# Continue with the container startup
exec gosu isaac-sim "$@"
