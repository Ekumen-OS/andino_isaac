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

# Final setup of the workspace

RUN mkdir -p $USER_WS/persistence && chmod 777 $USER_WS/persistence

COPY --chown=$USER:$GROUP docker/files/bash_aliases /home/$USER/.bash_aliases
COPY --link docker/entrypoint.sh /

ENV WITHIN_DEV=1

CMD        ["/bin/bash"]
ENTRYPOINT ["fixuid", "-q", "/entrypoint.sh"]
