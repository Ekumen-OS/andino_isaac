#!/bin/bash

set -e

# Prints information about usage.
function show_help() {
  echo $'\nUsage:\t build.sh [OPTIONS] \n
  Options:\n
  \t-i --image_name\t\t Name of the image to be built (default isaac_dev_docker).\n
  Example:\n
  \tbuild.sh --image_name custom_image_name\n'
}

echo "Building the docker image for NVidia Isaac development."

SCRIPT_FOLDER_PATH="$(cd "$(dirname "$0")"; pwd)"
CONTEXT_FOLDER_PATH="$(cd "$(dirname "$0")"; cd .. ; pwd)"

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -i|--image_name) IMAGE_NAME="${2}"; shift ;;
        -h|--help) show_help ; exit 1 ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

# Update the arguments to default values if needed.
OS_VERSION="jammy"
IMAGE_NAME=${IMAGE_NAME:-isaac_sim_ros2_humble_andino}
DOCKERFILE_PATH=$SCRIPT_FOLDER_PATH/Dockerfile

USERID=$(id -u)
USER=$(whoami)

sudo docker build -t $IMAGE_NAME \
     --file $DOCKERFILE_PATH \
     --build-arg USERID=$USERID \
     --build-arg USER=$USER \
     $CONTEXT_FOLDER_PATH
