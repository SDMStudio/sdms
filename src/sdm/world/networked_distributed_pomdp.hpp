// #pragma once

// #include <string>
// #include <vector>
// #include <list>
// #include <fstream>
// #include <iostream>
// #include <sstream>
// #include <map>
// #include <unordered_set>
// #include "math.h"
// #include <random>

// #include <sdm/types.hpp>
// #include <sdm/common.hpp>
// #include <sdm/public/world.hpp>
// // #include <sdm/public/feedback.hpp>
// #include <sdm/core/observation_dynamics.hpp>
// #include <sdm/core/state_dynamics.hpp>
// #include <sdm/core/space/discrete_space.hpp>
// #include <sdm/core/space/multi_discrete_space.hpp>
// #include <sdm/world/discrete_decpomdp.hpp>
// #include <sdm/utils/linear_algebra/vector.hpp>
// #include <sdm/utils/linear_algebra/matrix.hpp>

// namespace sdm
// {
//     class NetworkedDistributedPOMDP : public DiscreteDecPOMDP
//     {
//     private:
//         number internal_state_;

//     public:
//         class Node
//         {
//         public:
//             agent id;
//             int parent = -2;
//             double *upperBounds;
//             std::vector<agent> children;
//             std::unordered_set<agent> neighbors;
//             std::map<std::string, double> rewardFunction;
//             std::map<std::string, double> transitionFunction;
//             std::map<std::string, double> observationFunction;

//             Node();
//             Node(agent id, std::unordered_set<agent>);
//         };

//         action maxActions;

//         number num_agents_;

//         // observation observations;
//         // Vector startingBelief;

//         std::map<int, int> transitionmatrix;
//         std::unordered_set<state> *stateSuccessor;
//         std::map<std::string, double> observationsmatrix;
//         std::map<std::string, std::unordered_set<observation>> observationSuccessor;

//         /**
//          * @brief dynamics generator
//          */
//         std::unordered_map<std::string, std::discrete_distribution<number>> ndpomdp_dynamics_generator;

//         Node *n;

//         agent *nodes;

//         agent root;

//         std::ifstream input_file;

//         double rMax = 45;

//         NetworkedDistributedPOMDP(std::string);

//         void createDAG();
//         void printDAG(agent);
//         virtual state init();
//         void getData(std::string);
//         state getPGenerator(state);
//         void setupDynamicsGenerator();

//         /**
//          * @fn std::tuple<double, observation, state> getDynamicsGenerator(state x, action a)
//          * @param state the current state
//          * @param jaction the joint action
//          * @return a tuple containing reward, next_osbservation and next_state
//          **/
//         std::tuple<std::vector<double>, observation, state> getDynamicsGenerator(number, number);

//         std::vector<std::pair<number, number>> getUniqueValidNeighbors();

//         double getInitialBelief(state);
//         double getP(state, action, state);
//         virtual void execute(action, feedback *);

//         observation getQGenerator(agent, action, state);
//         std::unordered_set<state> getStateSuccessor(state);

//         double getRewardF(state, agent, agent, action, action);
//         double getObservation(agent, action, state, observation);
//         double getQ(state, agent, action, observation, agent, action, observation);

//         std::unordered_set<observation> getObservationSuccessor(agent, action, state);
//         std::tuple<state, observation, observation> getDynamicsGenerator(state, agent, action, agent, action);

//         virtual number getNumStates() const;

//         /**
//          * @brief Get the number of observations for a specific agent
//          * 
//          * @return number 
//          */
//         virtual number getNumObservations(number) const;

//         /**
//          * \brief Get the number of Actions for a specific agent
//          */
//         number getNumActions(number) const;

//         /**
//          * \brief Get the number of Action for every agents
//          */
//         std::vector<number> getNumActions() const;
//     };
// } // namespace sdm
