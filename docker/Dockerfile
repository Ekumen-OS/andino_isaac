# Download base image
FROM nvcr.io/nvidia/isaac-sim:2023.1.1

# Arguments for building
ARG USERID
ARG USER

# Setup environment
ENV TERM linux
ENV DEBIAN_FRONTEND noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Install ROS Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update
RUN apt upgrade -y

# Copy installation scripts, and perform the dependency installation.
COPY docker/install.sh .
COPY docker/requirements.txt .
RUN ./install.sh
RUN rm install.sh requirements.txt

# Creates the src folder of the workspace.
RUN mkdir -p /ws/src

# Adds the rosdep update to the entrypoint.
RUN rosdep init && rosdep update

# # Adds to bashrc the ros humble overlay sourcing.
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc


# Defines a workspace folder.
WORKDIR /ws

CMD ["/bin/bash"]
