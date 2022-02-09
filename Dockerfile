FROM ubuntu:20.04
LABEL maintainer="Ignacio Vizzo <ignaciovizzo@gmail.com>"

# setup environment
ENV TERM xterm
ENV DEBIAN_FRONTEND=noninteractive

# Install C++ dependencies and google protobuf
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    cmake \
    libprotobuf-dev \
    protobuf-compiler \
    && rm -rf /var/lib/apt/lists/*

# Install Python system dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    python3 \
    python3-dev \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --upgrade black
