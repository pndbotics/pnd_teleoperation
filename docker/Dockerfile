FROM osrf/ros:humble-desktop

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

# Clone and build CasADi
RUN cd /tmp && \
    git clone https://github.com/casadi/casadi.git casadi && \
    cd casadi && \
    git checkout 3.6.7 && \
    mkdir build && \
    cd build && \
    cmake -DWITH_PYTHON=ON -DWITH_IPOPT=ON -DWITH_OPENMP=ON -DWITH_THREAD=ON .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig

# Clean up
# RUN rm -rf /tmp/casadi

# Set the working directory
WORKDIR /pnd_workspace

