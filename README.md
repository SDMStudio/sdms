[![SDMS Logo](https://raw.githubusercontent.com/SDMStudio/sdms/main/docs/sdms-logo-black2.png)](https://sdmstudio.github.io)
------------------------------------------------------


[![License](https://img.shields.io/github/license/sdmstudio/sdms)](https://github.com/SDMStudio/sdms/blob/main/LICENSE)
[![Version](https://img.shields.io/github/v/tag/sdmstudio/sdms)](https://github.com/SDMStudio/sdms/tags)
[![Contributors](https://img.shields.io/github/contributors-anon/sdmstudio/sdms)](https://github.com/SDMStudio/sdms/graphs/contributors)


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
    - [Solve a problem (with planning algorithm)](#solve-a-problem-with-planning-algorithm)
    - [Solve a problem (with learning algorithm)](#solve-a-problem-with-learning-algorithm)
    - [Test a saved policy [TO DO]](#test-a-saved-policy-to-do)
- [4. Get started](#4-get-started)
- [5. TO DO](#5-to-do)
    - [Serial PWLCQ](#serial-pwlcq)
    - [NDPOMDP](#ndpomdp)
    - [Structure](#structure)


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

|        HSVI        |     Q-Learning     |  Value Iteration   |         A*         | Policy Iteration | JESP  |
| :----------------: | :----------------: | :----------------: | :----------------: | :--------------: | :---: |
| :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |       :x:        |  :x:  |


# 2. Installation

Follow installation instructions on https://sdmstudio.github.io/tutorials/install 

(aurelien : ) if u have issues with toulbar2 (error : cannot link -ltb2), then just put the .so of ltb2 that is in lib/ in /usr/lib.

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

### Solve a problem (with planning algorithm)
```bash
SDMStudio solve [ARG...]
SDMStudio solve [-a ALGO] [-p PROBLEM] [-f FORMALISM] [-e ERROR] [-d DISCOUNT] [-h HORIZON] [-t TRIALS] [-n EXP_NAME]
SDMStudio solve [--algorithm ALGO] [--problem PROBLEM] [--formalism FORMALISM] [--error ERROR] [--discount DISCOUNT] [--horizon HORIZON] [--trials TRIALS] [--name EXP_NAME]
```
**Exemple:** solve the multi-agent problem called *tiger* as it was a single-agent problem. HSVI will be used by default. 
```bash
cd sdms/
SDMStudio solve -p data/world/dpomdp/tiger.dpomdp -f pomdp -e 0.001 -d 1.0 -h 4
```

### Solve a problem (with learning algorithm)
```bash
SDMStudio learn [ARG...]
SDMStudio learn [-a ALGO] [-p PROBLEM] [-f FORMALISM] [-l LEARNING_RATE] [-d DISCOUNT] [-h HORIZON] [-t NUM_TIMESTEPS] [-n EXP_NAME]
SDMStudio learn [--algorithm ALGO] [--problem PROBLEM] [--formalism FORMALISM] [--lr LEARNING_RATE] [--discount DISCOUNT] [--horizon HORIZON] [--nb_timesteps NUM_TIMESTEPS] [--name EXP_NAME]
```

**Exemple:** solve the multi-agent problem called *tiger* as it was a single-agent problem. Q-learning will be used by default. 
```bash
cd sdms/
SDMStudio learn -p data/world/dpomdp/tiger.dpomdp -f pomdp -l 0.01 -d 1.0 -h 4 -t 30000 
```

### Test a saved policy [TO DO]
```bash
SDMStudio test [ARG...]
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

# 5. TO DO

### Serial PWLCQ
- Garder état simultané lié à l'état séquentialisé
- Faire équivalence sur l'état simultané
- Stocker a^{\kappa} lié au s^{\kappa}_{t} et le s^{\kappa}_{t+1} qui en résulte
- Ne mettre à jour le a^kappa qu'à une fréquence fixé
- Troncature = m donc à horizon infini on a que m+1 pas de temps 
- 

### NDPOMDP
- faire en sorte de pouvoir charger n'importe quel problème simultané 
- pouvoir le sérialiser
- factoriser la représentation des états d'occupations et des structures de représentation (fonction de valeur, dynamique, reward)

### Structure 
- déplacer les définitions vers les states
- transition et récompense
- Récompense : 
  - produit scalaire 
  - ou récompense vectoriel
- Transition :

- SG, ZS-SG, ZS-POSG, POSG