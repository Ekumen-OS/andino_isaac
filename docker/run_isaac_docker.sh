#!/bin/bash

DOCKERDIR="$( cd "$( dirname "$0" )" && pwd )"
PKGDIR=$DOCKERDIR/..


xhost +local:root
docker run -it --rm --privileged \
	--entrypoint bash \
	-e DISPLAY \
	-e "ACCEPT_EULA=Y" \
	-e "PRIVACY_CONSENT=Y" \
	-e "TERM=xterm-256color" \
	-v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	-v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
	-v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
	-v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
	-v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
	-v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
	-v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
	-v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
	-v ~/docker/isaac-sim/documents:/root/Documents:rw \
	-v $PKGDIR/:/ws/src/andino_isaac \
	--name isaac-sim \
	--gpus all \
	--network host \
	isaac_dev_docker:latest
