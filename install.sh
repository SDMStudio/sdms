#!/bin/bash
# This script will install SDM'Studio on this computer. 
# To this end, the script will install dependencies, build and install SDMS.
# Accepts three parameters: torch_url, cplex_root, proc
# Usage: ./install.sh --install_prefix=</PATH/TO/INSTALL/DIR> --torch_url=<URL/TO/PYTORCH.zip> --cplex_root=<PATH/TO/CPLEX_Studio> --proc=<NumProc>


# Variable declaration
CPLEX_ROOT="/opt/ibm/ILOG/CPLEX_Studio201/" # default is "/opt/ibm/ILOG/CPLEX_Studio201/"
INSTALL_PREFIX="$HOME/.sdms" # default is "$HOME/.sdms"
TORCH_URL='' # default is pytorch for CPU
PROC='4' # the default number of processors used is 4
 
BLUE='\033[1;36m'
NC='\033[0m' # No Color
LOG_SDMS="${BLUE}SDMS#>${NC} "

# Parse arguments
for i in "$@"; do
  case $i in
    -i=*|--install_prefix=*)
      INSTALL_PREFIX="${i#*=}"
      shift # past argument=value
      ;;
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
      echo -e "Usage: ./install.sh --install_prefix=<PATH/TO/INSTALLATION/DIR> --torch_url=<URL/TO/PYTORCH.zip> --cplex_root=<PATH/TO/CPLEX_Studio> --proc=<NumProc>"
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
    Darwin*)    machine=Darwin;;
    CYGWIN*)    machine=Cygwin;;
    MINGW*)     machine=MinGw;;
    *)          machine="UNKNOWN:${unameOut}"
esac

if [ "${machine}" == "Darwin" ] || [ "${machine}" == "Linux" ]; # Install SDMS under Mac OS X platform 
then
    SUDO=''
    if (( $EUID != 0 )); then
        SUDO='sudo'
    fi
   
    if [ "${machine}" == "Linux" ]; then
        echo -e "${LOG_SDMS}Starting installation on Linux platform"
        declare -a list_dependencies=( "libboost-all-dev" "libfmt-dev" "libgmp-dev" "zlib1g-dev" "liblzma-dev" "wget" "unzip" "cmake" "clang" )
    else   
        echo -e "${LOG_SDMS}Starting installation on Mac OS X platform"
        declare -a list_dependencies=( "boost" "fmt" "gmp" "zlib" "unzip" "wget" "cmake" "llvm" )
    fi
 
    echo -e "${LOG_SDMS}Install dependencies"

    # Install dependencies
    for dependency in ${list_dependencies[@]}; do
        echo -ne "- $dependency : "
        if [ $(dpkg-query -W -f='${Status}' ${dependency} 2>/dev/null | grep -c "ok installed") -eq 0 ];
        then
            if [ "${machine}" == "Linux" ]; then
                $SUDO apt-get install -y ${dependency};
            else   
                brew install ${dependency};
            fi
        else
            echo -e "installed"
        fi
    done

    # Install PyTorch
    if [ "${machine}" == "Linux" ]; then
        url_libtorch='https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.7.1%2Bcpu.zip'
    else   
        url_libtorch='https://download.pytorch.org/libtorch/cpu/libtorch-macos-1.9.0.zip'
    fi
    [[ -n ${TORCH_URL} ]] && url_libtorch=${TORCH_URL}

    echo -ne "- pytorch : "
    if [ ! -d /opt/libtorch ]; then
        echo -e "\n${LOG_SDMS}Download PyTorch from $url_libtorch"
        wget "$url_libtorch" -O /tmp/tmp_libtorch.zip
        $SUDO unzip /tmp/tmp_libtorch.zip -d /opt && rm /tmp/tmp_libtorch.zip
    else
        echo -e "installed"
    fi

    # Install other libs : Hard copy Toolbar library and other required libs
    echo -ne "- toolbar and boost_parser (building by sdms) : "
    mkdir -p ${INSTALL_PREFIX}/lib && cp lib/* ${INSTALL_PREFIX}/lib
    echo -e "installed"

    # Create build directory
    echo -e "${LOG_SDMS}Create build directory."
    rm -rf build
    mkdir -p ./build && cd ./build
  
    # if [ "${machine}" == "Linux" ]; then
    #     echo -e "${LOG_SDMS}Modify environment variable PATH."
    #     export PATH="${INSTALL_PREFIX}/bin:$PATH"
    #     echo "export PATH=${INSTALL_PREFIX}/bin:"'$PATH' >> ~/.bashrc

    #     echo -e "${LOG_SDMS}Modify environment variable LD_LIBRARY_PATH."
    #     export LD_LIBRARY_PATH="${INSTALL_PREFIX}/lib:$LD_LIBRARY_PATH"
    #     echo "export LD_LIBRARY_PATH=${INSTALL_PREFIX}/lib:"'$LD_LIBRARY_PATH' >> ~/.bashrc

    #     # alias brc='source ~/.bashrc'
    # else   
    #     echo -e "${LOG_SDMS}Modify environment variable PATH."
    #     export PATH="${INSTALL_PREFIX}/bin:$PATH"
    #     echo "export PATH=${INSTALL_PREFIX}/bin:"'$PATH' >> ~/.bashrc

    #     echo -e "${LOG_SDMS}Modify environment variable DYLD_LIBRARY_PATH."
    #     export DYLD_LIBRARY_PATH="${INSTALL_PREFIX}/lib:$DYLD_LIBRARY_PATH"
    #     echo "export DYLD_LIBRARY_PATH=${INSTALL_PREFIX}/lib:"'$DYLD_LIBRARY_PATH' >> ~/.bashrc
        
    #     # alias brc='source ~/.bash_profile'
    # fi

    # Build and install SDM'Studio
    echo -e "${LOG_SDMS}Build and install SDM'Studio."
    cmake -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" -DCMAKE_BUILD_TYPE=Release -DCPLEX_ROOT_DIR="${CPLEX_ROOT}" .. 
    make -j ${PROC} install

    # Check problem during SDMS installation
    RESULT_INSTALL_SDMS=$?
    if [ ${RESULT_INSTALL_SDMS} -eq 0 ];
    then
        echo -e "${LOG_SDMS}Installation completed. Use 'sdms --help' to see how to use it."
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