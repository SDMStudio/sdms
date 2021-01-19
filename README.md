![SDMS Logo](https://gitlab.inria.fr/chroma1/plasma/sdms/-/blob/develop/docs/theme/sdms_theme/static/img/sdms-icon.png)

--------------------------------------------------------------------------------

SDM'Studio: The Reconstruction ToolKit
======================================

<!-- [![Build Status](https://travis-ci.com/hill-a/stable-baselines.svg?branch=master)](https://travis-ci.com/hill-a/stable-baselines) 
[![Documentation Status](https://readthedocs.org/projects/stable-baselines/badge/?version=master)](https://stable-baselines.readthedocs.io/en/master/?badge=master) 
[![Codacy Badge](https://api.codacy.com/project/badge/Grade/3bcb4cd6d76a4270acb16b5fe6dd9efa)](https://www.codacy.com/app/baselines_janitors/stable-baselines?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=hill-a/stable-baselines&amp;utm_campaign=Badge_Grade) 
[![Codacy Badge](https://api.codacy.com/project/badge/Coverage/3bcb4cd6d76a4270acb16b5fe6dd9efa)](https://www.codacy.com/app/baselines_janitors/stable-baselines?utm_source=github.com&utm_medium=referral&utm_content=hill-a/stable-baselines&utm_campaign=Badge_Coverage)

[![GitHub release](https://img.shields.io/github/release/SimonRit/RTK.svg)](https://github.com/SimonRit/RTK/releases/latest) -->
<!-- [![PyPI](https://img.shields.io/pypi/v/itk-rtk.svg)](https://pypi.python.org/pypi/itk-rtk) -->
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://gitlab.inria.fr/chroma1/plasma/sdms/-/blob/main/LICENSE)

- [SDM'Studio: The Reconstruction ToolKit](#sdmstudio-the-reconstruction-toolkit)
  - [1. Requirements](#1-requirements)
  - [2. Installation](#2-installation)
    - [2.1. Quick install](#21-quick-install)
    - [2.2. Step by step installation](#22-step-by-step-installation)
  - [3. Docker Image](#3-docker-image)
    - [3.1. Using pre-built images](#31-using-pre-built-images)
    - [3.1. Building the image yourself](#31-building-the-image-yourself)
- [images are tagged as docker.io/${your_docker_username}/pytorch](#images-are-tagged-as-dockerioyour_docker_usernamepytorch)
  - [3. Formalisms](#3-formalisms)
    - [3.1. Multi-agent](#31-multi-agent)
    - [3.2. Single-agent](#32-single-agent)
  - [4. Algorithms](#4-algorithms)
  - [5. Usage](#5-usage)


## 1. Requirements
  - c++		    version >= 5.4.0
  - clang++ 	version >= 3.8.0
  - boost 	  version >= 1.66
  - eigen 	  version >= 3.0.0

## 2. Installation

### 2.1. Quick install
In order to execute `install.sh` file, you may need to change permissions using `chmod +x install.sh`.
```bash
  git clone https://gitlab.inria.fr/chroma1/plasma/sdms.git
  cd sdms
  ./install.sh
```

### 2.2. Step by step installation
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

## 3. Docker Image
### 3.1. Using pre-built images

You can also pull a pre-built docker image from Docker Hub and run with docker v19.03+

docker run --gpus all --rm -ti --ipc=host pytorch/pytorch:latest

Please note that PyTorch uses shared memory to share data between processes, so if torch multiprocessing is used (e.g. for multithreaded data loaders) the default shared memory segment size that container runs with is not enough, and you should increase shared memory size either with --ipc=host or --shm-size command line options to nvidia-docker run.

### 3.1. Building the image yourself

NOTE: Must be built with a docker version > 18.06

The Dockerfile is supplied to build images with Cuda support and cuDNN v7. You can pass PYTHON_VERSION=x.y make variable to specify which Python version is to be used by Miniconda, or leave it unset to use the default.

make -f docker.Makefile
# images are tagged as docker.io/${your_docker_username}/pytorch



## 3. Formalisms

### 3.1. Multi-agent
|        POSG        |     Dec-POMDP      |       ZSPOSG       |      NDPODMP       |         SG         | Dec-MDP |
| :----------------: | :----------------: | :----------------: | :----------------: | :----------------: | :-----: |
| :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |   :x:   |


### 3.2. Single-agent
|       POMDP        |        MDP         |
| :----------------: | :----------------: |
| :heavy_check_mark: | :heavy_check_mark: |

## 4. Algorithms

|        HSVI        | Q-Learning | Value Iteration | Policy Iteration | JESP  |
| :----------------: | :--------: | :-------------: | :--------------: | :---: |
| :heavy_check_mark: |    :x:     |       :x:       |       :x:        |  :x:  |


## 5. Usage

```cpp
#include <iostream>
#include <sdm/worlds.hpp>
#include <sdm/parser/parser.hpp>

int main(int argc, char **argv)
{
	sdm::DecPOMDP dpomdp_world= sdm::parser::parse_file("my_problem.dpomdp");
  
  std::cout << "Nb States : " << dpomdp_world.getNumStates() << std::endl;
	std::cout << "Nb Agents : " << dpomdp_world.getNumAgents() << std::endl;
	std::cout << "Nb Joint Actions : " << dpomdp_world.getNumJActions() << std::endl;
	std::cout << "Nb Joint Observations : " << dpomdp_world.getNumJObservations() << std::endl;

  return 0;
}
```
