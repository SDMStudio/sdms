#!/bin/bash
# This script will open a development environment with Docker. 
# Usage: ./open-docker.sh

BLUE='\033[1;36m'
NC='\033[0m' # No Color
LOG_SDMS="${BLUE}SDMS#>${NC} "

defaultDockerImage="blavad/sdms:latest"

echo -ne "${LOG_SDMS}Name of the branch to install (default is the current branch) : "
read branchName
if [ "$branchName" != "" ]
then 
    git checkout ${branchName}
fi
# Run the latest docker image (blavad/sdms:0.6-cpu-devel at this time)
echo -ne "${LOG_SDMS}Name of the docker image (default : \"${defaultDockerImage}\") : "
read dockerImage
if [ "$dockerImage" != "" ]
then 
    docker run  -ti --rm --mount type=bind,source="$(pwd)",target=/sdms -w "/sdms" $dockerImage
else
    docker run  -ti --rm --mount type=bind,source="$(pwd)",target=/sdms -w "/sdms" $defaultDockerImage
fi