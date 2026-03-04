FROM nvcr.io/nvidia/isaac-sim:5.1.0 AS build

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

ARG UID=1000
ARG GID=1000
# Remove ubuntu user/group if they own HOST_UID/HOST_GID
RUN if getent passwd ${UID:-1000} >/dev/null; then \
    USERNAME="$(getent passwd ${UID:-1000} | cut -d: -f1)"; \
    userdel -r "$USERNAME" || true; \
    fi; \
    \
    if getent group ${GID:-1000} >/dev/null; then \
    GROUPNAME="$(getent group ${GID:-1000} | cut -d: -f1)"; \
    groupdel "$GROUPNAME" || true; \
    fi; \
    \
    # Assign UID/GID to isaac-sim
    groupmod -g ${GID:-1000} isaac-sim; \
    usermod  -u ${UID:-1000} -g ${GID:-1000} isaac-sim;

# Use multistage to avoid Docker history double the image size
FROM ubuntu:24.04 AS andino_isaac
COPY --from=build / /

# Use user and group that comes with the Isaac base container
ARG USER=isaac-sim

USER ${USER}

ENV USER_WS=/workspaces/andino_isaac
WORKDIR $USER_WS

COPY --link docker/entrypoint.sh /.entrypoint.sh
ENTRYPOINT ["/.entrypoint.sh"]
CMD ["/bin/bash"]
