#!/bin/bash
# This script will install SDM'Studio on this computer. 
# To this end, the script will install dependencies, build and install SDMS.
# Accepts three parameters: torch_url, cplex_root, proc
# Usage: ./install.sh --torch_url=<URL/TO/PYTORCH.zip> --cplex_root=<PATH/TO/CPLEX_Studio> --proc=<NumProc>


# Variable declaration
CPLEX_ROOT='' # default is "/opt/ibm/ILOG/CPLEX_Studio201/"
TORCH_URL='' # default is pytorch for CPU
PROC='4' # the default number of processors used is 4
 
BLUE='\033[1;36m'
NC='\033[0m' # No Color
LOG_SDMS="${BLUE}SDMS#>${NC} "

# Parse arguments
for i in "$@"; do
  case $i in
    -t=*|--torch_url=*)
      TORCH_URL="${i#*=}"
      shift # past argument=value
      ;;
    -c=*|--cplex_root=*)
      CPLEX_ROOT="${i#*=}"
      shift # past argument=value
      ;;
    -p=*|--proc=*)
      PROC="${i#*=}"
      shift # past argument=value
      ;;
    -h|--help)
      echo -e "Usage: ./install.sh --torch_url=<URL/TO/PYTORCH.zip> --cplex_root=<PATH/TO/CPLEX_Studio> --proc=<NumProc>"
      echo -e "-c,--cplex_root \n\tPath to the cplex directory (/opt/ibm/ILOG/CPLEX_Studio201/)"
      echo -e "-h,--help \n\tShow this help"
      echo -e "-p,--proc \n\tNumber of processors used during installation"
      echo -e "-t,--torch_url \n\tUrl of the pytorch version (https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.7.1%2Bcpu.zip)"
      exit 0
      shift # past argument with no value
      ;;
    -*|--*)
      echo "Unknown option $i"
      exit 1
      ;;
    *)
      ;;
  esac
done


# Get the platform name
unameOut="$(uname -s)"
case "${unameOut}" in
    Linux*)     machine=Linux;;
    Darwin*)    machine=Mac;;
    CYGWIN*)    machine=Cygwin;;
    MINGW*)     machine=MinGw;;
    *)          machine="UNKNOWN:${unameOut}"
esac

if [ "${machine}" == "Darwin" ]; # Install SDMS under Mac OS X platform 
then
    echo -e "${LOG_SDMS}Starting installation on Mac OS X platform"
    echo -e "${LOG_SDMS}Install dependencies"

    # Install dependencies
    brew install eigen boost fmt gmp zlib zma unzip wget cmake clang
    # port install eigen boost libfmt unzip wget

    # Install PyTorch 
    url_libtorch='https://download.pytorch.org/libtorch/cpu/libtorch-macos-1.9.0.zip'
    [[ -n ${TORCH_URL} ]] && url_libtorch=${TORCH_URL}
    
    if [ ! -d /opt/libtorch ]; then
        echo -e "\n${LOG_SDMS}Download PyTorch from $url_libtorch"
        wget "$url_libtorch" -O /tmp/tmp_libtorch.zip && unzip /tmp/tmp_libtorch.zip -d /opt && rm /tmp/tmp_libtorch.zip
    else
        echo -e "${LOG_SDMS}PyTorch already installed"
    fi

    # Hard copy Toolbar library and other required libs
    cp lib/* /usr/lib/x86_64-linux-gnu/

    # Create build directory
    echo -e "${LOG_SDMS}Create build directory."
    rm -rf build
    mkdir -p ./build && cd ./build

    # Build and install SDM'Studio
    echo -e "${LOG_SDMS}Build and install SDM'Studio."
    if [ -n ${CPLEX_ROOT} ]; then
        cmake -DCMAKE_BUILD_TYPE=Release -DCPLEX_ROOT_DIR="${CPLEX_ROOT}" .. && make -j ${PROC} install
    else
        cmake -DCMAKE_BUILD_TYPE=Release .. && make -j ${PROC} install
    fi

    # Check problem during SDMS installation
    RESULT_INSTALL_SDMS=$?
    if [ ${RESULT_INSTALL_SDMS} -eq 0 ];
    then
        echo -e "${LOG_SDMS}Installation completed. Use 'SDMStudio --help' to see how to use it."
    else
        echo -e "${LOG_SDMS}Something went wrong (code ${RESULT_INSTALL_SDMS}) when executing \"cmake .. && make -j install\""
        echo -e "${LOG_SDMS}Installation will stop" && exit $?
    fi

elif [ "${machine}" == "Linux" ]; # Install SDMS under GNU/Linux platform
then
    declare -a dependencies=("libboost-all-dev" "libfmt-dev" "libgmp-dev" "zlib1g-dev" "liblzma-dev" "wget" "unzip" "cmake" "clang" )#libeigen3-dev
 
    echo -e "${LOG_SDMS}Starting installation on Linux platform"
    echo -e "${LOG_SDMS}Install dependencies"

    # Install dependencies
    for dependency in ${dependencies[@]}; do
        echo -ne "- $dependency : "
        if [ $(dpkg-query -W -f='${Status}' $dependency 2>/dev/null | grep -c "ok installed") -eq 0 ];
        then
            apt-get install -y ${dependency};
        else
            echo -e "installed"
        fi
    done

    # Install PyTorch
    url_libtorch='https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.7.1%2Bcpu.zip'
    [[ -n ${TORCH_URL} ]] && url_libtorch=${TORCH_URL}

    echo -ne "- pytorch : "
    if [ ! -d /opt/libtorch ]; then
        echo -e "\n${LOG_SDMS}Download PyTorch from $url_libtorch"
        wget "$url_libtorch" -O /tmp/tmp_libtorch.zip && unzip /tmp/tmp_libtorch.zip -d /opt && rm /tmp/tmp_libtorch.zip
    else
        echo -e "installed"
    fi

    # Hard copy Toolbar library and other required libs
    cp lib/* /usr/lib/x86_64-linux-gnu/

    # Create build directory
    echo -e "${LOG_SDMS}Create build directory."
    rm -rf build
    mkdir -p ./build && cd ./build

    # Build and install SDM'Studio
    echo -e "${LOG_SDMS}Build and install SDM'Studio."
    if [ -n ${CPLEX_ROOT} ]; then
        cmake -DCMAKE_BUILD_TYPE=Release -DCPLEX_ROOT_DIR="${CPLEX_ROOT}" .. && make -j ${PROC} install
    else
        cmake -DCMAKE_BUILD_TYPE=Release .. && make -j ${PROC} install
    fi

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