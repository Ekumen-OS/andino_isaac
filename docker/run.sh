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

docker compose run ${BUILD} --rm andino_isaac --remove-orphans
