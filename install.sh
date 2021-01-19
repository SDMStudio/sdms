#!/bin/bash
# This script will install SDM'Studio on this computer. 
# To this end, the script will install dependencies, build and install SDMS.
# Accepts one parameter: url_to_pytorch
# Usage: ./install.sh 

if test $# -gt 0
then
# Install dependencies
apt-get -y update && apt-get install -y libeigen3-dev libboost-all-dev unzip wget

# Download pytorch and install it in /opt/
echo "Call --> wget "$1
wget "$1" -O /tmp/tmp_libtorch.zip && unzip /tmp/tmp_libtorch.zip -d /opt && rm /tmp/tmp_libtorch.zip
mkdir -p ./build && cd ./build

# Build and install SDMS
cmake .. -DCMAKE_PREFIX_PATH=/opt/libtorch && make install
fi
