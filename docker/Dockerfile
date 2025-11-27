ARG ISAAC_SIM_VERSION=5.1.0

FROM ros:humble-ros-base-jammy AS cacher

WORKDIR /ws/src

COPY . andino_isaac/

RUN mkdir -p /tmp/ws/src \
    && find ./ -name "package.xml" | xargs cp --parents -t /tmp/ws/src \
    && find ./ -name "COLCON_IGNORE" | xargs cp --parents -t /tmp/ws/src \
    || true

FROM nvcr.io/nvidia/isaac-sim:${ISAAC_SIM_VERSION} AS andino_isaac

ENV ROS_DISTRO=humble

ENV TERM=linux
ENV DEBIAN_FRONTEND=noninteractive

USER root

# APT requirements
COPY --link docker/requirements.txt /apt-requirements.txt
RUN apt-get update && \
    apt-get install --no-install-recommends -y $(cat /apt-requirements.txt) && \
    rm -rf /var/lib/apt/lists/*

RUN rm /apt-requirements.txt

# Add the ROS 2 key and install a base ROS 2 installation
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update \
    && apt install --no-install-recommends -y \
    --allow-downgrades libbrotli1=1.0.9-2build6 \
    libbrotli-dev \
    libfreetype6-dev \
    libfontconfig1-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-${ROS_DISTRO}-desktop \
    && rm -rf /var/lib/apt/lists/*

USER $USER:$GROUP

# Use default user that the Isaac sim 5.1 image uses
ARG USER=isaac-sim
ARG GROUP=isaac-sim

ARG LOCAL_UID=1000
ARG LOCAL_GID=1000
RUN set -eux; \
    if getent passwd ${LOCAL_UID} >/dev/null; then \
    echo "Removing existing user with UID ${LOCAL_UID}"; \
    userdel -f $(getent passwd ${LOCAL_UID} | cut -d: -f1) || true; \
    fi; \
    if getent group ${LOCAL_GID} >/dev/null; then \
    echo "Removing existing group with GID ${LOCAL_GID}"; \
    groupdel $(getent group ${LOCAL_GID} | cut -d: -f1) || true; \
    fi;

RUN usermod -u ${LOCAL_UID} -o isaac-sim \
    && groupmod -g ${LOCAL_GID} -o isaac-sim

ENV USER_WS=/home/$USER/ws

RUN mkdir -p $USER_WS/src
WORKDIR $USER_WS

# Install the project dependencies
COPY --from=cacher --chown=$USER:$GROUP /tmp/ws/ $USER_WS/
RUN /bin/bash -c '\
    sudo apt update \
    && sudo rosdep init \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && rosdep update \
    && rosdep install -i -y --from-path src \
    && sudo rm -rf /var/lib/apt/lists/*'

# Install fixuid to setup the correct uid in the files when starting the container
COPY --chown=root:root docker/files/fixuid_config.yml /etc/fixuid/config.yml
RUN /bin/bash -c '\
    ARCH=`uname -m` && if [ "$ARCH" == "aarch64" ]; then FIXUID_ARCH="arm64"; else FIXUID_ARCH="amd64"; fi \
    && curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.6.0/fixuid-0.6.0-linux-$FIXUID_ARCH.tar.gz | sudo tar -C /usr/local/bin -xzf - \
    && sudo chmod 4755 /usr/local/bin/fixuid \
    && cd /etc/fixuid \
    && sudo sed -i "s/_USER_/$USER/" config.yml \
    && sudo sed -i "s/_GROUP_/$GROUP/" config.yml \
    && echo Done installing fixuid for $FIXUID_ARCH'

#
# Final setup of the workspace

RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USER/.bashrc

RUN mkdir -p $USER_WS/persistence && chmod 777 $USER_WS/persistence

COPY --chown=$USER:$GROUP docker/files/bash_aliases /home/$USER/.bash_aliases
COPY --link docker/entrypoint.sh /

ENV WITHIN_DEV=1

CMD        ["/bin/bash"]
ENTRYPOINT ["fixuid", "-q", "/entrypoint.sh"]
