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
- [3. Basic Usage](#3-basic-usage)
    - [List available algorithms](#list-available-algorithms)
    - [List available worlds](#list-available-worlds)
    - [Solve a problem](#solve-a-problem)
    - [Test a saved policy [TO DO]](#test-a-saved-policy-to-do)
- [4. Get started](#4-get-started)


# 1. About SDM'Studio

## 1.1. Formalisms

### Multi-agent
| POSG  |     Dec-POMDP      | ZSPOSG |      NDPODMP       |  SG   | Dec-MDP |
| :---: | :----------------: | :----: | :----------------: | :---: | :-----: |
|  :x:  | :heavy_check_mark: |  :x:   | :heavy_check_mark: |  :x:  |   :x:   |


### Single-agent
|       POMDP        |        MDP         |
| :----------------: | :----------------: |
| :heavy_check_mark: | :heavy_check_mark: |

## 1.2. Algorithms

|        HSVI        |     Q-Learning     |  Value Iteration   | Policy Iteration | JESP  |
| :----------------: | :----------------: | :----------------: | :--------------: | :---: |
| :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |       :x:        |  :x:  |


# 2. Installation

Follow installation instructions on https://aldavid.gitlabpages.inria.fr/sdms/tutorials/install .

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
