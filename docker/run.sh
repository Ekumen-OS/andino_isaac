#!/usr/bin/env bash

sudo xhost +

mkdir -p ./docker/isaac-sim/cache/main
mkdir -p ./docker/isaac-sim/cache/computecache
mkdir -p ./docker/isaac-sim/logs
mkdir -p ./docker/isaac-sim/config
mkdir -p ./docker/isaac-sim/data
mkdir -p ./docker/isaac-sim/pkg

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

LOCAL_UID=$(id -u) LOCAL_GID=$(id -g) docker compose run ${BUILD} --rm andino_isaac
