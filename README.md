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


## Installing

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

# TO DO

## Avant JMLR

***Planning semaine pro***
- Lundi/Mardi : résoudre les croix (1) et (2) + tester horizon infini et le faire fonctionner
- Mercredi/Jeudi matin : Finir la vérif d'installation Mac et mettre au propre la partie tuto installation via docker
- Jeudi Après-midi/Vendredi/Lundi/.. : Vérifier amélioration serial, debugguer puis lancer expés 

### A debugguer

|           |   (tab) VI ou PBVI  |     (tab) HSVI     |  (tab) QLearning   | (pwlc) VI ou PBVI  |  (pwlc/saw) HSVI   |  (pwlc) QLearning  |           action_selection           | Horizon infini |
| :-------: | :----------------: | :----------------: | :----------------: | :----------------: | :----------------: | :----------------: | :----------------: | :----------------------------------: |
|    MDP    |         -          | :heavy_check_mark: | :heavy_check_mark: |      :x: (1)       |         -          |         -          |         -          |              tab-exhaus              | non testé      |
|   bMDP    | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |      :x: (3)       |       tab-exhaus, pwlc-exhaus        | non testé      |
|   oMDP    |         -          | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |  tab-exhaus, pwlc-exhaus, pwlc-wscp  | non testé      |
| hier-MDP  |         -          | :heavy_check_mark: | :heavy_check_mark: |      :x: (1)       |         -          |         -          |         -          |              tab-exhaus              | non testé      |
| hier-bMDP | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |    :x: (2 ou 3)    |    :x: (2 ou 3)    |      :x: (3)       |       tab-exhaus, pwlc-exhaus        | non testé      |
| hier-oMDP |         -          |      :x: (2)       |      :x: (2)       | :heavy_check_mark: |      :x: (2)       |      :x: (2)       | :heavy_check_mark: |  tab-exhaus, pwlc-exhaus, pwlc-wscp  | non testé      |
|  ext-MDP  |         -          | :heavy_check_mark: | :heavy_check_mark: |      :x: (1)       |         -          |         -          |         -          |              tab-exhaus              | non testé      |
| ext-bMDP  | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |      :x: (3)       |       tab-exhaus, pwlc-exhaus        | non testé      |
| ext-oMDP  |         -          | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | tab-exhaus, pwlc-exhaus, pwlc-serial | non testé      |

**(1)** segmentation fault de Q-learning tabulaire pour les MDPs (difficulté facile)

**(2)** hierarchical oMDP ne converge pas vers la bonne valeur avec HSVI -- possiblement lié à la définition erronée de HierarchicalMPOMDP::getReachableObservations (difficulté moyenne)

**(3)** pas de PWLCQUpdate défini pour les beliefMDP (difficulté moyenne)

### A modifier
- [x] Modifier UpdateOperator to UpdateRule
- [x] Bouger algos dans algorithms/planning ou algorithms/rl
- [x] Tester BackwardInduction (et code example)
- [ ] Sauvagarder et charger fonction de valeur

### A finir 
- [ ] cas horizon infini 
  - besoin de le tester dans les différents cas (algos/structures)
- [ ] configuration TOML (--> simplification de la déclaration des problèmes)
  - me paraît intéressant pour simplifier le paramétrage d'SDMS mais peut attendre la semaine après soumission 
- [ ] (ext-oMDP -> oext-MDP) utiliser référence sur états simultanés suivant pour le greedy et backup
  - je ne suis pas 100% sûre de l'implémentation - à vérifier avant dans lancer les expés
- [x] installation Mac OSX  

### A tester 

- [ ] Avoir un `make test`  qui test sur de petits horizons différentes bench et algo.
- [ ] Tester perfs ext-oMDP avec PWLCQ (ancienne version vs nouvelle).

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
