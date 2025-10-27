FROM nvcr.io/nvidia/isaac-sim:5.1.0 AS andino_isaac

# Use user and group that comes with the Isaac base container
ARG USER=isaac-sim
ARG GROUP=isaac-sim

ENV TERM=linux
ENV DEBIAN_FRONTEND=noninteractive

USER root

# apt requirements
COPY --link docker/requirements.txt /apt-requirements.txt
RUN apt-get update && \
    apt-get install --no-install-recommends -y $(cat /apt-requirements.txt) && \
    rm -rf /var/lib/apt/lists/*
RUN rm /apt-requirements.txt

RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Remove existing user and group with ID 1000 in the container, if those exist
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

COPY --link docker/entrypoint.sh /.entrypoint.sh
ENTRYPOINT ["/.entrypoint.sh"]
CMD ["/bin/bash"]
