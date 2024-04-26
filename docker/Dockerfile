ARG ISAAC_SIM_VERSION=2023.1.1

# Download base image
FROM nvcr.io/nvidia/isaac-sim:${ISAAC_SIM_VERSION}

# Arguments for building
ARG USERID
ARG USER

# Setup environment
ENV TERM linux
ENV DEBIAN_FRONTEND noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Add ROS 2 key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update

# Copy installation scripts, and perform the dependency installation.
COPY docker/install.sh .
COPY docker/requirements.txt .
RUN ./install.sh
RUN rm install.sh requirements.txt

# Adds the rosdep update to the entrypoint.
RUN rosdep init && rosdep update

# Create a user with passwordless sudo
RUN adduser --uid $USERID --gecos "ekumen developer" --disabled-password $USER
RUN adduser $USER sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo "export QT_X11_NO_MITSHM=1" >> /home/$USER/.bashrc
USER $USER

# Adds USER to dialout and plugdev group.
# This is needed to access the serial ports, for further references check
# the libserial documentation.
RUN sudo usermod -a -G dialout $USER
RUN sudo usermod -a -G plugdev $USER

# Change isaac-sim's ownership
RUN sudo mkdir -p /isaac-sim/kit/exts/omni.gpu_foundation/cache/nv_shadercache
RUN sudo chown -R $USER:$USER /isaac-sim/kit/exts/omni.gpu_foundation/cache/nv_shadercache

# Creates the src folder of the workspace.
RUN mkdir -p /home/$USER/ws/src

# Adds to bashrc the ros humble overlay sourcing.
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USER/.bashrc
# Adds colcon autocomplete
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USER/.bashrc

# Defines a workspace folder.
WORKDIR /isaac-sim

# Add symlink
RUN sudo ln -s exts/omni.isaac.examples/omni/isaac/examples extension_examples

# Defines a workspace folder.
WORKDIR /home/$USER/ws

CMD ["/bin/bash"]
