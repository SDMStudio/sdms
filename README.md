<img alt="SDMS'Studio" src="https://raw.githubusercontent.com/SDMStudio/sdms/develop/docs/sdms-icon-gray.png" width="450">

[![License](https://img.shields.io/github/license/sdmstudio/sdms)](https://github.com/SDMStudio/sdms/blob/main/LICENSE)
[![Version](https://img.shields.io/github/v/tag/sdmstudio/sdms)](https://github.com/SDMStudio/sdms/tags)
[![Contributors](https://img.shields.io/github/contributors-anon/sdmstudio/sdms)](https://github.com/SDMStudio/sdms/graphs/contributors)


SDM'Studio is a C++ librairy that provides efficient solvers for sequential decision making problems.

## About SDM'Studio

### 1. Formalisms

| POSG (oMG) |  Dec-POMDP (oMDP)  | NDPODMP  (nd-oMDP) |    POMDP (bMDP)    |        MDP         |    serial-MMDP     |   serial-MPOMDP    | serial-Dec-POMDP (ser-oMDP) |
| :--------: | :----------------: | :----------------: | :----------------: | :----------------: | :----------------: | :----------------: | :-------------------------: |
|    :x:     | :heavy_check_mark: |        :x:         | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |     :heavy_check_mark:      |

### 2. Algorithms

|        HSVI        |     Q-Learning     |  Value Iteration   |         A*         | Policy Iteration | JESP  |
| :----------------: | :----------------: | :----------------: | :----------------: | :--------------: | :---: |
| :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |       :x:        |  :x:  |


## Installation

Follow instructions at https://sdmstudio.github.io/tutorials/install 

## Basic Usage

Several scripts are available after installing SDMS. The main program `sdms` should cover most of the basic usage. If, this is not enough, you may have a look to other SDMS programs `sdms-xxxx`.

### List available algorithms
```bash
sdms algorithms
```

### List available worlds
```bash
sdms worlds
```

### Solve a problem (with planning algorithm)
```bash
sdms solve [ARG...]
sdms solve [-a ALGO] [-p PROBLEM] [-f FORMALISM] [-e ERROR] [-d DISCOUNT] [-h HORIZON] [-t TRIALS] [-n EXP_NAME]
sdms solve [--algorithm ALGO] [--problem PROBLEM] [--formalism FORMALISM] [--error ERROR] [--discount DISCOUNT] [--horizon HORIZON] [--trials TRIALS] [--name EXP_NAME]
```
**Exemple:** solve the multi-agent problem called *tiger* as it was a single-agent problem. HSVI will be used by default. 
```bash
cd sdms/
sdms solve -a hsvi -p tiger.dpomdp -f bMDP -e 0.001 -h 10 -d 1.0
sdms solve -a qlearning -w tiger.dpomdp -f bMDP -h 10 -d 1.0 -t 30000 
```

### Test a saved policy [TO DO]
```bash
sdms test [ARG...]
```

## Uninstalling

Linux users can run `cat install_manifest.txt | xargs -d '\n' rm` as root from the build directory to uninstall SDM'Studio from their system.
