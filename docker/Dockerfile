ARG ISAAC_SIM_VERSION=5.1.0

FROM nvcr.io/nvidia/isaac-sim:${ISAAC_SIM_VERSION} AS andino_isaac

ENV TERM=linux
ENV DEBIAN_FRONTEND=noninteractive

USER root

# APT requirements
COPY --link docker/requirements.txt /apt-requirements.txt
RUN apt-get update && \
    apt-get install --no-install-recommends -y $(cat /apt-requirements.txt) && \
    rm -rf /var/lib/apt/lists/*

RUN rm /apt-requirements.txt

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

WORKDIR $USER_WS

RUN mkdir -p $USER_WS/persistence && chmod 777 $USER_WS/persistence

COPY --link docker/entrypoint.sh /

ENV ROS_DISTRO=jazzy
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/isaac-sim/python.sh", "./src/andino_isaac/tools/isaac_launch_script.py", "--world_file", "./src/andino_isaac/isaac_worlds/plain_world.usda", "--robot_file", "./src/andino_isaac_description/andino.usd"]
