
# TO DO

### A debugguer

|           |   (tab) VI ou PBVI  |     (tab) HSVI     |  (tab) QLearning   | (pwlc) VI ou PBVI  |  (pwlc/saw) HSVI   |  (pwlc) QLearning  |           action_selection           | (tab) Horizon infini |
| :-------: | :----------------: | :----------------: | :----------------: | :----------------: | :----------------: | :----------------: | :----------------: | :----------------------------------: |
|    MDP    |      :heavy_check_mark: | :heavy_check_mark: |      :heavy_check_mark:       |         -          |         -          |         -          |              tab-exhaus              |:heavy_check_mark:      |
|   bMDP    |  :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |      :x: (3)       |       tab-exhaus, pwlc-exhaus        |    :heavy_check_mark:   |
|   oMDP    |     :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |  tab-exhaus, pwlc-exhaus, pwlc-wscp  |:heavy_check_mark:      |
| hier-bMDP | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |    :x: (2 ou 3)    |    :x: (2 ou 3)    |      :x: (3)       |       tab-exhaus, pwlc-exhaus        | :heavy_check_mark:    |
| hier-oMDP |          :heavy_check_mark:       |      :heavy_check_mark:     | :heavy_check_mark: |      :x: (2)       |      :x: (2)       | :heavy_check_mark: |  tab-exhaus, pwlc-exhaus, pwlc-wscp  |:heavy_check_mark:    |
|  ext-MDP  |    :heavy_check_mark: | :heavy_check_mark: |      :heavy_check_mark:       |         -          |         -          |         -          |              tab-exhaus              | :x: (4)    |
| ext-bMDP  | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |      :x: (3)       |       tab-exhaus, pwlc-exhaus        |  :x: (4)      |
| ext-oMDP  |   :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | tab-exhaus, pwlc-exhaus, pwlc-serial |  :x: (4)      |

- [x] **(1)** segmentation fault de Q-learning tabulaire pour les MDPs (difficulté facile)

- [ ] **(2)** hierarchical oMDP ne converge pas vers la bonne valeur en PWLC -- possiblement lié à la définition erronée de HierarchicalMPOMDP::getReachableObservations (difficulté moyenne)

- [ ] **(3)** pas de PWLCQUpdate défini pour les beliefMDP (difficulté moyenne)

- [ ] **(4)** la plateforme gère l'horizon infini comme un horizon égal à 0. On ne peut pas sérialiser un horizon de 0 autrement qu'en disant qu'il s'agit de l'horizon *n_agents*. Mais si on fait cela, ce n'est plus infini.


### Modifications / Ajouts
- [x] Modifier UpdateOperator to UpdateRule
- [x] Bouger algos dans algorithms/planning ou algorithms/rl
- [x] Tester BackwardInduction (et code example)
- [ ] Sauvagarder et charger fonction de valeur
- [x] Parser spécifique pour MDP, POMDP, MMDP, MPOMDP, POSG
- [ ] Cas horizon infini 
  - besoin de le tester dans les différents cas (algos/structures)
- [ ] Configuration TOML (--> simplification de la déclaration des problèmes)
  - me paraît intéressant pour simplifier le paramétrage d'SDMS mais peut attendre la semaine après soumission 
- [ ] (ext-oMDP -> oext-MDP) utiliser référence sur états simultanés suivant pour le greedy et backup
  - je ne suis pas 100% sûre de l'implémentation - à vérifier avant dans lancer les expés
- [x] installation Mac OSX  

### Tests

- [x] Avoir un `make test`  qui test sur de petits horizons différentes bench et algo.
- [ ] Tester perfs ext-oMDP avec PWLCQ (ancienne version vs nouvelle).

### A documenter
- [ ] Beaucoup de choses 

## Pour la suite

### Discussion sur l'amélioration d'architecture logiciel

Architecture logicielle actuelle: 
- Chaque module gère un traitement spécifique 
  - `fonction de valeur` : stocke des valeurs par rapport à des états
  - `update rule` : mets à jour un état étant donné une action
- Certains modules sont génériques et peuvent utiliser les fonctions actuelles (de base) du state mais d'autres opérateurs sont particuliers à un type d'état et donc on voudrait 


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


## Doc 

### Update Rule

Appel à la fonction de valeur V qui appelle les données sur l'état s pour récupérer la les 

### Select Action
