#!/bin/bash

ARG USER

DOCKERDIR="$( cd "$( dirname "$0" )" && pwd )"
PKGDIR=$DOCKERDIR/..

# Nvidia's instructions indicates that this also needs to be mounted:
#   -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
# We do not include it because it caused launch issues:
#   https://github.com/Ekumen-OS/andino_isaac/pull/19#issuecomment-2159105512

xhost +local:root
docker run -it --rm --privileged \
	--entrypoint bash \
	-e DISPLAY \
	-e "ACCEPT_EULA=Y" \
	-e "PRIVACY_CONSENT=Y" \
	-e "TERM=xterm-256color" \
	-v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	-v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
	-v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
	-v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
	-v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
	-v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
	-v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
	-v ~/docker/isaac-sim/documents:/root/Documents:rw \
	-v $PKGDIR/:/home/$USER/ws/src/andino_isaac \
	--name isaac-sim \
	--gpus all \
	--network host \
	isaac_sim_ros2_humble_andino:latest
