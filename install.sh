#!/bin/bash
# This script will install SDM'Studio on this computer. 
# To this end, the script will install dependencies, build and install SDMS.
# Accepts one parameter: url_to_pytorch
# Usage: ./install.sh <URL/TO/PYTORCH.zip>

BLUE='\033[1;36m'
NC='\033[0m' # No Color
LOG_SDMS="${BLUE}SDMS#>${NC} "

# Get the platform
unameOut="$(uname -s)"
case "${unameOut}" in
    Linux*)     machine=Linux;;
    Darwin*)    machine=Mac;;
    CYGWIN*)    machine=Cygwin;;
    MINGW*)     machine=MinGw;;
    *)          machine="UNKNOWN:${unameOut}"
esac

if [ "${machine}" == "Darwin" ]; 
then
    # Install SDMS under Mac OS X platform 
    echo -e "${LOG_SDMS}Starting installation on Mac OS X platform"
    # Install dependencies
    echo -e "${LOG_SDMS}Install dependencies"
    brew install eigen boost fmt unzip wget
    # port install eigen boost libfmt unzip wget

    # Download pytorch and install it in /opt/
    url_libtorch='https://download.pytorch.org/libtorch/cpu/libtorch-macos-1.9.0.zip'
    if test $# -gt 0
    then
        echo -e "${LOG_SDMS}Download pytorch - custom path = $1"
        wget "$1" -O /tmp/tmp_libtorch.zip && unzip /tmp/tmp_libtorch.zip -d /opt && rm /tmp/tmp_libtorch.zip
    else
        echo -e "${LOG_SDMS}Download pytorch - default path = $url_libtorch"
        wget "$url_libtorch" -O /tmp/tmp_libtorch.zip && unzip /tmp/tmp_libtorch.zip -d /opt && rm /tmp/tmp_libtorch.zip
    fi
    mkdir -p ./build && cd ./build

    # Build and install SDMS
    cmake .. -DGLOBAL=ON && make install
    # Check problem during SDMS installation
    RESULT_INSTALL_SDMS=$?
    if [ ${RESULT_INSTALL_SDMS} -eq 0 ];
    then
        echo -e "${LOG_SDMS}Installation completed. Use 'SDMStudio --help' to see how to use it."
    else
        echo -e "${LOG_SDMS}Something went wrong (code ${RESULT_INSTALL_SDMS}) when executing \"cmake .. -DGLOBAL=ON && make install\""
        echo -e "${LOG_SDMS}Installation will stop" && exit $?
    fi


elif [ "${machine}" == "Linux" ]; 
then
    # Install SDMS under GNU/Linux platform
    echo -e "${LOG_SDMS}Starting installation on Linux platform"
    # Install dependencies
    echo -e "${LOG_SDMS}Install dependencies"
    apt-get -y update && apt-get install -y libeigen3-dev libboost-all-dev libfmt-dev unzip wget 

    # Download pytorch and install it in /opt/
    url_libtorch='https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.7.1%2Bcpu.zip'
    if test $# -gt 0
    then
        echo -e "${LOG_SDMS}Download pytorch - custom path = $1"
        wget "$1" -O /tmp/tmp_libtorch.zip && unzip /tmp/tmp_libtorch.zip -d /opt && rm /tmp/tmp_libtorch.zip
    else
        echo -e "${LOG_SDMS}Download pytorch - default path = $url_libtorch"
        wget "$url_libtorch" -O /tmp/tmp_libtorch.zip && unzip /tmp/tmp_libtorch.zip -d /opt && rm /tmp/tmp_libtorch.zip
    fi
    mkdir -p ./build && cd ./build

    # Build and install SDMS
    cmake .. -DGLOBAL=ON && make install
    # Check problem during SDMS installation
    RESULT_INSTALL_SDMS=$?
    if [ ${RESULT_INSTALL_SDMS} -eq 0 ];
    then
        echo -e "${LOG_SDMS}Installation completed. Use 'SDMStudio --help' to see how to use it."
    else
        echo -e "${LOG_SDMS}Something went wrong (code ${RESULT_INSTALL_SDMS}) when executing \"cmake .. -DGLOBAL=ON && make install\""
        echo -e "${LOG_SDMS}Installation will stop" && exit $?
    fi
elif [ "${machine}" == "MINGW32_NT" ]; 
then
    # Do something under 32 bits Windows NT platform
    echo -e "${LOG_SDMS}SDM'Studio is not available for 32 bits Windows NT platform"
elif [ "${machine}" == "MINGW64_NT" ]; 
then
    # Do something under 64 bits Windows NT platform
    echo -e "${LOG_SDMS}SDM'Studio is not available for 64 bits Windows NT platform"
fi


