ARG ISAAC_SIM_VERSION=4.2.0

# Download base image
FROM nvcr.io/nvidia/isaac-sim:${ISAAC_SIM_VERSION}

# Arguments for building
ARG USER=dev
ARG GROUP=ekumen

# Setup environment
ENV TERM=linux
ENV DEBIAN_FRONTEND=noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Add ROS 2 key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update

# Install apt requirements
COPY --link docker/requirements.txt /requirements.txt
RUN apt-get update && apt-get install --no-install-recommends -y $(cat /requirements.txt) && rm -rf /var/lib/apt/lists/*
RUN rm /requirements.txt

# Create a user with passwordless sudo
RUN addgroup $GROUP && adduser --ingroup $GROUP --disabled-password --gecos "" $USER && \
    adduser $USER sudo && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

USER $USER:$GROUP

# Change isaac-sim's ownership
RUN sudo mkdir -p /isaac-sim/kit/exts/omni.gpu_foundation/cache/nv_shadercache
RUN sudo chown -R $USER:$GROUP /isaac-sim/kit/exts/omni.gpu_foundation/cache/nv_shadercache

# Creates the src folder of the workspace.
RUN mkdir -p /home/$USER/ws/src

# Adds to bashrc the ros humble overlay sourcing.
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USER/.bashrc
# Adds colcon autocomplete
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USER/.bashrc

# Adds the rosdep update to the entrypoint.
RUN sudo rosdep init && rosdep update && sudo apt update

# Defines a workspace folder.
WORKDIR /home/$USER/ws

COPY --link docker/entrypoint.sh /.entrypoint.sh
ENTRYPOINT ["/.entrypoint.sh"]
