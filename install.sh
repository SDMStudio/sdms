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
    brew install eigen boost fmt gmp zlib zma unzip wget cmake clang
    # port install eigen boost libfmt unzip wget

    # Download pytorch and install it in /opt/
    url_libtorch='https://download.pytorch.org/libtorch/cpu/libtorch-macos-1.9.0.zip'
    
    # If Pytorch is not installed, we install it.
    if [ ! -d /opt/libtorch ]; then
        # If the user provide a custom argument
        [ $# -gt 0 ] && url_libtorch=$1

        echo -e "${LOG_SDMS}Download PyTorch from $1"
        wget "$url_libtorch" -O /tmp/tmp_libtorch.zip && unzip /tmp/tmp_libtorch.zip -d /opt && rm /tmp/tmp_libtorch.zip
    else
        echo -e "${LOG_SDMS}PyTorch already installed"
    fi

    mkdir -p ./build && cd ./build

    # Build and install SDMS
    cmake -DCMAKE_BUILD_TYPE=Release .. && make -j install
    # Check problem during SDMS installation
    RESULT_INSTALL_SDMS=$?
    if [ ${RESULT_INSTALL_SDMS} -eq 0 ];
    then
        echo -e "${LOG_SDMS}Installation completed. Use 'SDMStudio --help' to see how to use it."
    else
        echo -e "${LOG_SDMS}Something went wrong (code ${RESULT_INSTALL_SDMS}) when executing \"cmake .. && make -j install\""
        echo -e "${LOG_SDMS}Installation will stop" && exit $?
    fi


elif [ "${machine}" == "Linux" ]; 
then
    # Install SDMS under GNU/Linux platform
    echo -e "${LOG_SDMS}Starting installation on Linux platform"
    # Install dependencies
    echo -e "${LOG_SDMS}Install dependencies"
    apt-get -y update && apt-get install -y libeigen3-dev libboost-all-dev libfmt-dev libgmp-dev zlib1g-dev liblzma-dev wget unzip cmake clang
    # Default URL for libtorch on LINUX
    url_libtorch='https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.7.1%2Bcpu.zip'
   
    # If Pytorch is not installed, we install it.
    if [ ! -d /opt/libtorch ]; then
        # If the user provide a custom argument
        [ $# -gt 0 ] && url_libtorch=$1

        echo -e "${LOG_SDMS}Download PyTorch from $1"
        wget "$url_libtorch" -O /tmp/tmp_libtorch.zip && unzip /tmp/tmp_libtorch.zip -d /opt && rm /tmp/tmp_libtorch.zip
    else
        echo -e "${LOG_SDMS}PyTorch already installed"
    fi

    # Install Toolbar library 
    cp lib/libtb2.so /lib/x86_64-linux-gnu/

    echo -e "${LOG_SDMS}Create build directory."
    mkdir -p ./build && cd ./build

    echo -e "${LOG_SDMS}Build and install SDM'Studio."
    cmake -DCMAKE_BUILD_TYPE=Release .. && make -j install

    # Check problem during SDMS installation
    RESULT_INSTALL_SDMS=$?
    if [ ${RESULT_INSTALL_SDMS} -eq 0 ];
    then
        echo -e "${LOG_SDMS}Installation completed. Use 'SDMStudio --help' to see how to use it."
    else
        echo -e "${LOG_SDMS}Something went wrong (code ${RESULT_INSTALL_SDMS}) when executing \"cmake .. && make -j install\""
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


