FROM ubuntu:noble
LABEL authors="murilomarinho"
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Toolchain, C++ dependency and Debian packaging tools. qpOASES is vendored as
# a git submodule, so only Eigen needs to come from the archive.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential g++ cmake make pkg-config \
        libeigen3-dev \
        dpkg-dev debhelper devscripts \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/solver-qpoales
COPY . /opt/solver-qpoales

CMD ["bash", "/opt/solver-qpoales/docker/build-deb.sh"]
