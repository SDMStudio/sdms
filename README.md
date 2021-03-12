![SDMS Logo](https://gitlab.inria.fr/chroma1/plasma/sdms/raw/develop/docs/theme/sdms_theme/static/img/sdms-icon-sm.png)
---------------------------------------------------------------------------------------------------------------------------------

<!-- [![Build Status](https://travis-ci.com/hill-a/stable-baselines.svg?branch=master)](https://travis-ci.com/hill-a/stable-baselines) 
[![Documentation Status](https://readthedocs.org/projects/stable-baselines/badge/?version=master)](https://stable-baselines.readthedocs.io/en/master/?badge=master) 
[![Codacy Badge](https://api.codacy.com/project/badge/Grade/3bcb4cd6d76a4270acb16b5fe6dd9efa)](https://www.codacy.com/app/baselines_janitors/stable-baselines?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=hill-a/stable-baselines&amp;utm_campaign=Badge_Grade) 
[![Codacy Badge](https://api.codacy.com/project/badge/Coverage/3bcb4cd6d76a4270acb16b5fe6dd9efa)](https://www.codacy.com/app/baselines_janitors/stable-baselines?utm_source=github.com&utm_medium=referral&utm_content=hill-a/stable-baselines&utm_campaign=Badge_Coverage)

[![GitHub release](https://img.shields.io/github/release/SimonRit/RTK.svg)](https://github.com/SimonRit/RTK/releases/latest) -->
<!-- [![PyPI](https://img.shields.io/pypi/v/itk-rtk.svg)](https://pypi.python.org/pypi/itk-rtk) -->
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://gitlab.inria.fr/chroma1/plasma/sdms/-/blob/main/LICENSE)

SDM'Studio is a C++ librairy that provides efficient solvers for sequential decision making problems.

- [1. About SDM'Studio](#1-about-sdmstudio)
  - [1.1. Formalisms](#11-formalisms)
    - [Multi-agent](#multi-agent)
    - [Single-agent](#single-agent)
  - [1.2. Algorithms](#12-algorithms)
- [2. Installation](#2-installation)
  - [2.1. Quick install](#21-quick-install)
  - [2.2. Step by step installation](#22-step-by-step-installation)
  - [2.3 Docker Image](#23-docker-image)
    - [Using pre-built images](#using-pre-built-images)
    - [Building the image yourself](#building-the-image-yourself)
    - [For developers](#for-developers)
    - [Grid'5000 users](#grid5000-users)
- [3. Basic Usage](#3-basic-usage)
    - [List available algorithms](#list-available-algorithms)
    - [List available worlds](#list-available-worlds)
    - [Solve a problem](#solve-a-problem)
    - [Test a saved policy [TO DO]](#test-a-saved-policy-to-do)
- [4. Get started](#4-get-started)


# 1. About SDM'Studio

## 1.1. Formalisms

### Multi-agent
|        POSG        |     Dec-POMDP      |       ZSPOSG       |      NDPODMP       |         SG         | Dec-MDP |
| :----------------: | :----------------: | :----------------: | :----------------: | :----------------: | :-----: |
| :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |   :x:   |


### Single-agent
|       POMDP        |        MDP         |
| :----------------: | :----------------: |
| :heavy_check_mark: | :heavy_check_mark: |

## 1.2. Algorithms

|        HSVI        | Q-Learning | Value Iteration | Policy Iteration | JESP  |
| :----------------: | :--------: | :-------------: | :--------------: | :---: |
| :heavy_check_mark: |    :x:     |       :x:       |       :x:        |  :x:  |


# 2. Installation

## 2.1. Quick install
In order to execute `install.sh` file, you may need to change permissions using `chmod +x install.sh`.
```bash
  git clone https://gitlab.inria.fr/chroma1/plasma/sdms.git
  cd sdms
  sudo ./install.sh
```
You can pass an parameter of the form `path/to/libtorch-xxxxx.zip` to specify which  configuration of PyTorch is to be used.
```bash
  sudo ./install.sh path/to/libtorch-xxxxx.zip
```

## 2.2. Step by step installation
**Install SDMS dependencies**

```bash
  sudo apt-get install clang libeigen3-dev libboost-all-dev
```
**Install pytorch**

Download the last version of PyTorch C++ for cxx11 ABI according to your machine requirements (https://pytorch.org/get-started/locally/) and unzip the downloaded file into `/opt/` directory.
```bash
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-xxxxxxx.zip -O libtorch.zip
unzip libtorch.zip -d /opt
```
**Build and install SDM'Studio**
```bash
mkdir -p build && cd build
cmake .. -DCMAKE_PREFIX_PATH=/opt/libtorch
make install
```

## 2.3 Docker Image

### Using pre-built images

You can also pull a pre-built docker image from Docker Hub and run with docker. See available tags on [DockerHub](https://hub.docker.com/r/blavad/sdms).
```bash
docker run --rm -ti blavad/sdms:<tag>
docker run --rm -ti blavad/sdms:latest
```

### Building the image yourself

The `Dockerfile` is supplied to build images with PyTorch for CPU. You can pass `LIBTORCH_URL=<path/to/libtorch-xxxxx.zip` argument to specify which  configuration of PyTorch is to be used.

```bash
docker build --rm -t sdms:<tag> .
```

Then you can run a container and get an interactive command on the newly built image.

```bash
docker run --rm -ti sdms:<tag>
```

### For developers

Developers can use the multi-stage build architecture to improve flexibility during development process. If you don't want to install SDMS dependencies on your machine or you want to avoid building a new docker image each time you make changes in the code, you can build and run the base `dev` image using `bind mount` tools to mount your local repo in a container. The following command lines should do that: 

```bash
docker build --target dev -t sdms:develop .
docker run -ti --name sdms-dev --mount type=bind,source="$(pwd)",target=/home/sdms sdms:devel
```

With custom parameters, it is possible to build an image that is configure to work with any required version of CUDA.
```bash
docker build --build-arg BASE_IMAGE=nvidia/cuda:<tag> --build-arg LIBTORCH_URL=<url/to/cuda/libtorch> --target dev -t sdms:<tag> .

docker build --build-arg BASE_IMAGE=nvidia/cuda:10.2-cudnn7-devel-ubuntu18.04 --build-arg LIBTORCH_URL=https://download.pytorch.org/libtorch/cu102/libtorch-cxx11-abi-shared-with-deps-1.7.1.zip --target dev -t blavad/sdms:0.1-cuda10.2-cudnn7-devel .
```

### Grid'5000 users

Grid'5000 users can follow the procedure below to run experiments with GPUs on Grid'5000.

```bash
# Connect to a site on grid'5000
ssh (site).g5k

# Get SDMS sources on g5k with the way you prefer (git clone, scp or rsync)
git clone https://gitlab.inria.fr/chroma1/plasma/sdms
cd sdms/

# Reserve a node with GPUs (params should be adapted to your needs)
oarsub -p "cluster='cluster-name'" -I

# Setup CUDA and CUDNN for Docker in the interactive node
g5k-setup-nvidia-docker -t
module load cudnn

# Pull the docker image that is adapted for your usage (on g5k must be 'cuda10.1-cudnn8') 
docker pull blavad/sdms:<version> # ex: docker pull blavad/sdms:0.1-cuda10.1-cudnn8-devel

# (alternatively you can build the image yourself)
# docker build --build-arg BASE_IMAGE=nvidia/cuda:10.1-cudnn8-devel-ubuntu18.04 --build-arg LIBTORCH_URL=https://download.pytorch.org/libtorch/cu101/libtorch-cxx11-abi-shared-with-deps-1.8.0%2Bcu101.zip --target dev -t  sdms:0.1-cuda10.1-cudnn8-devel .

# Run the docker image interactively
docker run --rm --gpus all -ti --name sdms-dev  --mount type=bind,source="$(pwd)",target=/home/sdms blavad/sdms:<version>

# Run experiments on your needs 
```

# 3. Basic Usage

Several scripts are available after installing SDMS. The main program `SDMStudio` should cover most of the basic usage. If, this is not enough, you may have a look to other SDMS programs `sdms-xxxx`.

### List available algorithms
```bash
SDMStudio algorithms
```

### List available worlds
```bash
SDMStudio worlds
```

### Solve a problem
```bash
SDMStudio solve [ARG...]
SDMStudio solve [--algorithm ALGO] [--problem PROBLEM] [--formalism FORMALISM] [--error ERROR] [--discount DISCOUNT] [--horizon HORIZON] [--trials TRIALS]
```

### Test a saved policy [TO DO]
```bash
SDMStudio test [OPTIONS]
```

# 4. Get started

```cpp
#include <iostream>
#include <sdm/worlds.hpp>
#include <sdm/parser/parser.hpp>

int main(int argc, char **argv)
{
	auto dpomdp_world= sdm::parser::parse_file("my_problem.dpomdp");
  
	std::cout << "Nb Agents : " << dpomdp_world->getNumAgents() << std::endl;
  std::cout << "State Space : " << *dpomdp_world->getStateSpace() << std::endl;
	std::cout << "Action Space : " << *dpomdp_world->getActionSpace() << std::endl;
	std::cout << "Observation space : " << *dpomdp_world->getObsSpace() << std::endl;

  return 0;
}
```
