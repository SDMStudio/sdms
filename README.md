![SDMStudio](https://gitlab.inria.fr/chroma1/plasma/sdms/-/blob/develop/docs/theme/sdms_theme/static/img/sdms-icon.png)


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
- [3. Basic Usage](#3-basic-usage)
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
  ./install.sh
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

You can also pull a pre-built docker image from Docker Hub and run with docker
```bash
docker run --rm -ti blavad/sdms:latest
```

### Building the image yourself

The `Dockerfile` is supplied to build images with PyTorch for CPU. You can pass `LIBTORCH_URL=<path/to/libtorch-xxxxx.zip` argument to specify which  configuration of PyTorch is to be used.

```bash
docker build --rm -ti sdms:v1.0 .
```

# 3. Basic Usage

# 4. Get started

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
