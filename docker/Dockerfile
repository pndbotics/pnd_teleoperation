FROM osrf/ros:humble-desktop
ARG USER_UID=1000
ARG USER_GID=1000
ARG USER_NAME=dockeruser
# Create a group and user
# Only create if they don't already exist (e.g. in base image)
RUN if ! getent group ${USER_GID} > /dev/null 2>&1; then \
    groupadd --gid ${USER_GID} ${USER_NAME} || groupmod -g ${USER_GID} $(getent group ${USER_GID} | cut -d: -f1); \
    fi && \
    if ! getent passwd ${USER_UID} > /dev/null 2>&1; then \
    useradd --uid ${USER_UID} --gid ${USER_GID} --create-home --shell /bin/bash ${USER_NAME} || usermod -u ${USER_UID} $(getent passwd ${USER_UID} | cut -d: -f1); \
    fi

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Update and install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    coinor-libipopt-dev \
    gfortran \
    liblapack-dev \
    pkg-config \
    swig \
    git \
    cmake \
    python3 \
    python3-pip \
    --install-recommends && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# default activate ros environment
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USER_NAME}/.bashrc