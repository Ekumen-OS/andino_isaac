set +e

DEFAULT_IMAGE_NAME=isaac_sim_ros2_humble_andino
DEFAULT_CONTAINER_NAME=isaac_sim_ros2_humble_andino_container

# Prints information about usage.
function show_help() {
  cat << EOF
    Usage:    run.sh [OPTIONS]
    Options:
        -i --image_name         Name of the image to be run (default $DEFAULT_IMAGE_NAME).
        -c --container_name     Name of the container(default $DEFAULT_CONTAINER_NAME).
    Examples:
        run.sh
        run.sh --image_name custom_image_name --container_name custom_container_name
EOF
}

# Returns true when the path is relative, false otherwise.
#
# Arguments
#   $1 -> Path
function is_relative_path() {
  case $1 in
    /*) return 1 ;; # false
    *) return 0 ;;  # true
  esac
}

echo "Running the container..."

# Location of the repository
REPOSITORY_FOLDER_PATH="$(cd "$(dirname "$0")"; cd ..; pwd)"
REPOSITORY_FOLDER_NAME=$( basename $REPOSITORY_FOLDER_PATH )

DSIM_REPOS_PARENT_FOLDER_PATH="$(cd "$(dirname "$0")"; cd ..; pwd)"
# Location from where the script was executed.
RUN_LOCATION="$(pwd)"

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -i|--image_name) IMAGE_NAME="${2}"; shift ;;
        -c|--container_name) CONTAINER_NAME="${2}"; shift ;;
        -h|--help) show_help ; exit 1 ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

# Update the arguments to default values if needed.
IMAGE_NAME=${IMAGE_NAME:-$DEFAULT_IMAGE_NAME}
CONTAINER_NAME=${CONTAINER_NAME:-$DEFAULT_CONTAINER_NAME}

SSH_PATH=/home/$USER/.ssh
WORKSPACE_SRC_CONTAINER=/home/$(whoami)/ws/src/$REPOSITORY_FOLDER_NAME
WORKSPACE_ROOT_CONTAINER=/home/$(whoami)/ws
SSH_AUTH_SOCK_USER=$SSH_AUTH_SOCK

# Create cache folders to store colcon build files
mkdir -p ${REPOSITORY_FOLDER_PATH}/.build
mkdir -p ${REPOSITORY_FOLDER_PATH}/.install

# Transfer the ownership to the user
chown -R "$USER" ${REPOSITORY_FOLDER_PATH}/.build
chown -R "$USER" ${REPOSITORY_FOLDER_PATH}/.install

# Check if name container is already taken.
if sudo -g docker docker container ls -a | grep "${CONTAINER_NAME}$" -c &> /dev/null; then
   printf "Error: Docker container called $CONTAINER_NAME is already opened.     \
   \n\nTry removing the old container by doing: \n\t docker rm $CONTAINER_NAME   \
   \nor just initialize it with a different name.\n"
   exit 1
fi

xhost +
sudo docker run --privileged --net=host -it --entrypoint /bin/bash \
       --gpus=all -e NVIDIA_DRIVER_CAPABILITIES=all \
       -e DISPLAY=$DISPLAY \
       -e SSH_AUTH_SOCK=$SSH_AUTH_SOCK_USER \
       -e "ACCEPT_EULA=Y" \
       -v $(dirname $SSH_AUTH_SOCK_USER):$(dirname $SSH_AUTH_SOCK_USER) \
       -v /dev/dri:/dev/dri \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       -v $SSH_PATH:$SSH_PATH \
       -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
       -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
       -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
       -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
       -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
       -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
       -v ~/docker/isaac-sim/kit_logs:/isaac-sim/kit/logs
       -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
       -v ~/docker/isaac-sim/documents:/root/Documents:rw \
       -v ${REPOSITORY_FOLDER_PATH}:$WORKSPACE_SRC_CONTAINER \
       --name $CONTAINER_NAME $IMAGE_NAME
xhost -

# Trap workspace exits and give the user the choice to save changes.
function onexit() {
  while true; do
    read -p "Do you want to overwrite the image called '$IMAGE_NAME' with the current changes? [y/n]: " answer
    if [[ "${answer:0:1}" =~ y|Y ]]; then
      echo "Overwriting docker image..."
      sudo docker commit $CONTAINER_NAME $IMAGE_NAME
      break
    elif [[ "${answer:0:1}" =~ n|N ]]; then
      break
    fi
  done
  docker stop $CONTAINER_NAME > /dev/null
  docker rm $CONTAINER_NAME > /dev/null
}

trap onexit EXIT