#!/usr/bin/env bash

sudo xhost +

declare SCRIPT_NAME=$(readlink -f ${BASH_SOURCE[0]})
cd $(dirname $SCRIPT_NAME)

BUILD=""

if [[ ! -z "$1" ]]; then
    if [[ "$1" == "--build" || "$1" == "-b" ]]; then
        BUILD="--build"
    else
        echo "Unknown argument ${1}"
        exit 1
    fi
fi

# Create cache directories in host machine and give container user permissions
echo "Creating Isaac sim cache directories"
mkdir -p ~/docker/isaac-sim-andino/cache/main/ov
mkdir -p ~/docker/isaac-sim-andino/cache/main/warp
mkdir -p ~/docker/isaac-sim-andino/cache/computecache
mkdir -p ~/docker/isaac-sim-andino/config
mkdir -p ~/docker/isaac-sim-andino/data/documents
mkdir -p ~/docker/isaac-sim-andino/data/Kit
mkdir -p ~/docker/isaac-sim-andino/logs
mkdir -p ~/docker/isaac-sim-andino/pkg
sudo chown -R 1234:1234 ~/docker/isaac-sim-andino

LOCAL_UID=$(id -u) LOCAL_GID=$(id -g) docker compose run ${BUILD} --remove-orphans --rm andino_isaac

sudo xhost -
