<img alt="SDMS'Studio" src="https://raw.githubusercontent.com/SDMStudio/sdms/develop/docs/sdms-icon-gray.png" width="500">

[![License](https://img.shields.io/github/license/sdmstudio/sdms)](https://github.com/SDMStudio/sdms/blob/main/LICENSE)
[![Version](https://img.shields.io/github/v/tag/sdmstudio/sdms)](https://github.com/SDMStudio/sdms/tags)
[![Contributors](https://img.shields.io/github/contributors-anon/sdmstudio/sdms)](https://github.com/SDMStudio/sdms/graphs/contributors)


SDM'Studio is a C++ librairy that provides efficient solvers for sequential decision making problems.

## About SDM'Studio

### 1. Formalisms

| POSG (oMG) |     Dec-POMDP (oMDP)    |      NDPODMP  (nd-oMDP)     |      POMDP (bMDP)       |        MDP       | serial-MMDP | serial-MPOMDP | serial-Dec-POMDP (ser-oMDP) |
| :---: | :----------------: | :----------------: |:----------------: | :----------------: | :----------------: |:----------------: | :----------------: |
|  :x:  | :heavy_check_mark: |  :x: |  :heavy_check_mark: | :heavy_check_mark: |:heavy_check_mark: | :heavy_check_mark: |:heavy_check_mark: |

### 2. Algorithms

|        HSVI        |     Q-Learning     |  Value Iteration   |         A*         | Policy Iteration | JESP  |
| :----------------: | :----------------: | :----------------: | :----------------: | :--------------: | :---: |
| :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |       :x:        |  :x:  |


## Installation

Follow installation instructions : https://sdmstudio.github.io/tutorials/install 

## Basic Usage

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
SDMStudio solve -a hsvi -p tiger.dpomdp -f bMDP -e 0.001 -h 10 -d 1.0
SDMStudio solve -a qlearning -w tiger.dpomdp -f bMDP -h 10 -d 1.0 -t 30000 
```

### Test a saved policy [TO DO]
```bash
SDMStudio test [ARG...]
```

## Get started

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

## TO DO

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
