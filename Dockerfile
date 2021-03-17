ARG BASE_IMAGE=ubuntu:18.04

# Base image of SDMS for developers (contains all dependencies of sdms)
# --> developers can use this base combined with "bind mount" tool of docker to develop on their machine and test their implementation in container 
FROM ${BASE_IMAGE} AS dev
ARG LIBTORCH_URL=https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.7.1%2Bcpu.zip

RUN apt-get -y update \
    && apt-get install -y  \
    libeigen3-dev \
    libboost-all-dev \
    unzip \
    wget \
    clang \
    cmake \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /opt \
    && wget ${LIBTORCH_URL} -O tmp_libtorch.zip\
    && unzip tmp_libtorch.zip -d /opt\
    && rm tmp_libtorch.zip

# Make a snapshot of the source code and build an image based on this source code at this moment.
# The image contains the sources (/opt/sdms) and installed sdms base on these sources 
FROM dev AS build

COPY . /opt/sdms
WORKDIR /opt/sdms

RUN cmake . -DCMAKE_PREFIX_PATH=/opt/libtorch -DBUILD_DOCS=OFF -DBUILD_TESTS=OFF \
    && make install

# Image that contain SDMS source, documentation and tests. Everything is built and install in /usr/local.
# FROM dev-base AS build-all

# RUN apt-get -y update \
#     && apt-get install -y  \
#     doxygen \
#     python3-pip
#     python3-sphinx \

# RUN pip3 install breathe

# RUN cmake . -DCMAKE_PREFIX_PATH=/opt/libtorch -DBUILD_DOCS=ON -DBUILD_TESTS=ON \
#     && make install

# Same as build but does not contains sources of SDMS so that the size of the image is smaller
FROM dev AS runtime

COPY --from=build /usr/local /usr/local
ENV SDMS_PATH="/usr/local"
ENV PATH="${SDMS_PATH}/bin:${PATH}"
ENV LD_LIBRARY_PATH="${SDMS_PATH}/lib:${LD_LIBRARY_PATH}"