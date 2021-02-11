ARG BASE_IMAGE=ubuntu:18.04

# Base image of SDMS (contains all dependencies)
FROM ${BASE_IMAGE} AS dev-base
ARG LIBTORCH_URL=https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.7.1%2Bcpu.zip

RUN apt-get -y update \
    && apt-get install -y  \
    libeigen3-dev \
    libboost-all-dev \
    unzip \
    wget \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /opt \
    && wget ${LIBTORCH_URL} -O tmp_libtorch.zip\
    && unzip tmp_libtorch.zip -d /opt\
    && rm tmp_libtorch.zip

# Build image (minimal requirements to be able to build yourself SDMS in a container)
FROM dev-base AS build-base

RUN apt-get -y update \
    && apt-get install -y  \
    clang \
    cmake

COPY . /opt/sdms
WORKDIR /opt/sdms
  
# Image that contain SDMS source built
FROM build-base AS build

RUN cmake . -DCMAKE_PREFIX_PATH=/opt/libtorch -DBUILD_DOCS=OFF -DBUILD_TESTS=OFF \
    && make install

# Image that contain SDMS source, documentation and tests built
# FROM dev-base AS build-all

# RUN apt-get -y update \
#     && apt-get install -y  \
#     doxygen \
#     python3-pip
#     python3-sphinx \

# RUN pip3 install breathe

# RUN cmake . -DCMAKE_PREFIX_PATH=/opt/libtorch -DBUILD_DOCS=ON -DBUILD_TESTS=ON \
#     && make install

# Image that contain binary files
# FROM ${BASE_IMAGE} AS runtime
# COPY --from=build /usr/local /usr/local
# CMD [ "SDMStudio" ]