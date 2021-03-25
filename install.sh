#!/bin/bash
# This script will install SDM'Studio on this computer. 
# To this end, the script will install dependencies, build and install SDMS.
# Accepts one parameter: url_to_pytorch
# Usage: ./install.sh <URL/TO/PYTORCH.zip>

url_libtorch='https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.7.1%2Bcpu.zip'

# Install dependencies
apt-get -y update && apt-get install -y libeigen3-dev libboost-all-dev libfmt-dev unzip wget

# Download pytorch and install it in /opt/
if test $# -gt 0
then
    echo "Call --> wget "$1
    wget "$1" -O /tmp/tmp_libtorch.zip && unzip /tmp/tmp_libtorch.zip -d /opt && rm /tmp/tmp_libtorch.zip
else
    echo "Call --> wget "$url_libtorch
    wget "$url_libtorch" -O /tmp/tmp_libtorch.zip && unzip /tmp/tmp_libtorch.zip -d /opt && rm /tmp/tmp_libtorch.zip
fi
mkdir -p ./build && cd ./build

# Build and install SDMS
cmake .. -DCMAKE_PREFIX_PATH=/opt/libtorch && make install
echo "#> Installation completed. Use 'SDMStudio --help' to see how to use it."
