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

# TO DO

## Avant JMLR

### A debugguer

|           |  (tab) VI ou PBVI  |     (tab) HSVI     |  (tab) QLearning   | (pwlc) VI ou PBVI  |  (pwlc/saw) HSVI   |  (pwlc) QLearning  |           action_selection           |
| :-------: | :----------------: | :----------------: | :----------------: | :----------------: | :----------------: | :----------------: | :----------------------------------: |
|    MDP    | :heavy_check_mark: | :heavy_check_mark: |      :x: (1)       |         -          |         -          |         -          |              tab-exhaus              |
|   bMDP    | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |      :x: (3)       |       tab-exhaus, pwlc-exhaus        |
|   oMDP    | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |  tab-exhaus, pwlc-exhaus, pwlc-wscp  |
| hier-MDP  | :heavy_check_mark: | :heavy_check_mark: |      :x: (1)       |         -          |         -          |         -          |              tab-exhaus              |
| hier-bMDP | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |      :x: (2 ou 3)       |      :x: (2 ou 3)       |      :x: (3)       |       tab-exhaus, pwlc-exhaus        |
| hier-oMDP |      :x: (2)       |      :x: (2)       | :heavy_check_mark: |      :x: (2)       |      :x: (2)       | :heavy_check_mark: |  tab-exhaus, pwlc-exhaus, pwlc-wscp  |
|  ext-MDP  | :heavy_check_mark: | :heavy_check_mark: |      :x: (1)       |         -          |         -          |         -          |              tab-exhaus              |
| ext-bMDP  | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |      :x: (3)       |       tab-exhaus, pwlc-exhaus        |
| ext-oMDP  | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | tab-exhaus, pwlc-exhaus, pwlc-serial |

(1) segmentation fault de Q-learning tabulaire pour les MDPs (difficulté facile)
(2) hierarchical oMDP ne converge pas vers la bonne valeur avec HSVI -- possiblement lié à la définition erronée de HierarchicalMPOMDP::getReachableObservations (difficulté moyenne)
(3) pas de PWLCQUpdate défini pour les beliefMDP (difficulté moyenne)


### A finir
- [ ] installation Mac OSX 
- [ ] cas horizon infini 
- [ ] configuration TOML (--> simplification de la déclaration des problèmes)
- [ ] (ext-oMDP -> oext-MDP) utiliser référence sur états simultanés suivant pour le greedy et backup

### A tester perfs

- [ ] Tester perfs ext-oMDP avec PWLCQ (ancienne version vs nouvelle)

### A documenter
- [ ] Beaucoup de choses 

## Pour la suite

### Serial PWLCQ
<!-- - [] Garder état simultané lié à l'état séquentialisé
- [] Faire équivalence sur l'état simultané -->
- [ ] Stocker a^{\kappa} lié au s^{\kappa}_{t} et le s^{\kappa}_{t+1} qui en résulte
- [ ] Ne mettre à jour le a^kappa qu'à une fréquence fixée
- [x] Troncature = m donc à horizon infini on a que m+1 pas de temps 

### NDPOMDP
- [ ] faire en sorte de pouvoir charger n'importe quel problème simultané 
- [ ] pouvoir le sérialiser
- [ ] factoriser la représentation des états d'occupations et des structures de représentation (fonction de valeur, dynamique, reward)

### Structure 
- [x] déplacer les définitions vers les states
- [x] pouvoir parser POSG
