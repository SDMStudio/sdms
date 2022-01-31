// #pragma once

// #include <fstream>
// #include <iostream>
// #include <sstream>
// #include <random>
// #include <unordered_set>

// #include <sdm/types.hpp>
// #include <sdm/core/space/space.hpp>
// #include <sdm/world/base/mpomdp_interface.hpp>
// #include <sdm/utils/linear_programming/ndpomdp_naming.hpp>

// namespace sdm
// {
//     class NDPOMDP : public MPOMDPInterface, 
//                     public NDPOMDPNaming
//     {

//     public:
//         class Node
//         {
//         public:
//             number agent_id;
//             int parent = -2;
//             double *upperBounds;
//             std::vector<number> children;
//             std::unordered_set<number> neighbors;
//             std::map<std::string, double> rewardFunction;
//             std::map<std::string, double> transitionFunction;
//             std::map<std::string, double> observationFunction;

//             Node();
//             Node(number agent_id, std::unordered_set<number> neighbors);
//         };

//         NDPOMDP(std::string filename);

//         virtual std::shared_ptr<State> init();

//         void getData(std::string);

//         std::vector<std::pair<number, number>> getUniqueValidNeighbors();

//         double getInitialBelief(std::shared_ptr<State>);

//         double getObservation(number agent, number action, number state, number observation);

//         double getRewardF(number state_id, number agent_id1, number agent_id2, number action_id1, number action_id2) const;

//          /**
//          * @brief Get the number of agents
//          *
//          * @return the number of agents
//          */
//         number getNumAgents() const;

//         /**
//          * @brief Get the number of agents
//          *
//          * @return the number of agents
//          */
//         number getHorizon() const;

//         /**
//          * @brief Get the discount factor at timestep t.
//          *
//          * @param t the timestep
//          * @return the discount factor
//          */
//         double getDiscount(number t = 0) const;

//         /**
//          * @brief Get the weighted discount factor at timestep t.
//          *
//          * @param t the timestep
//          * @return the discount factor
//          */
//         double getWeightedDiscount(number t = 0) const;

//         /**
//          * @brief Get the initial distribution over states.
//          *
//          * @return the initial distribution over states
//          */
//         std::shared_ptr<Distribution<std::shared_ptr<State>>> getStartDistribution() const;

//         /**
//          * @brief Get ths state space at timestep t.
//          *
//          * @param t the timestep
//          * @return the state space
//          */
//         std::shared_ptr<Space> getStateSpace(number t = 0) const;

//         /**
//          * @brief Get ths action space at timestep t.
//          *
//          * @param t the timestep
//          * @return the action space
//          */
//         std::shared_ptr<Space> getActionSpace(number t = 0) const;

//         /**
//          * @brief Get the reward at timestep t when executing an action in a specific state.
//          *
//          * @param state the current state
//          * @param action the action
//          * @param t the timestep
//          * @return double the reward for each agent
//          */
//         double getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0) const;

//         double getMinReward(number t = 0) const;

//         double getMaxReward(number t = 0) const;

//         std::tuple<std::shared_ptr<State>, std::vector<double>, bool> step(std::shared_ptr<Action> action);

//         std::tuple<std::shared_ptr<State>, std::vector<double>, bool> step(std::shared_ptr<Action> action, bool increment_timestep);

//         /**
//          * @brief Get the transition probability, i.e. p(s' | s, a).
//          *
//          * @param state the current state
//          * @param action the action
//          * @param next_state the next state
//          * @param t the timestep
//          * @return the probability
//          */
//         double getTransitionProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t) const;

//         /**
//          * @brief Get reachable states
//          *
//          * @param state the current state
//          * @param action the current action
//          * @return the set of reachable states
//          */
//         std::set<std::shared_ptr<State>> getReachableStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const;


//         /**
//          * @brief Get ths observation space at timestep t.
//          * 
//          * @param t the timestep
//          * @return the observation space
//          */
//         std::shared_ptr<Space> getObservationSpace(number t) const;

//         /**
//          * @brief Get reachable observations
//          * 
//          * @param state the current state
//          * @param action the current action
//          * @return the set of reachable observations
//          */
//         std::set<std::shared_ptr<Observation>> getReachableObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t) const;

//         /**
//          * @brief Get the observation probability, i.e. p(o | s', a)
//          * 
//          * @param action the action
//          * @param next_state the next state
//          * @param observation the observation
//          * @param t the timestep
//          * @return the probability
//          */
//         double getObservationProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const;

//         /**
//          * @brief Get the dynamics, i.e. p(s', o | s, a)
//          * 
//          * @param state the state at timestep t
//          * @param action the action 
//          * @param next_state the next state, i.e. timestep t+1
//          * @param observation the observation
//          * @param t the timestep
//          * @return the probability
//          */
//         double getDynamics(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const;

//         void setInternalState(std::shared_ptr<State> state);

//         std::shared_ptr<State> getInternalState() const;


//         std::shared_ptr<MPOMDPInterface> getSubMPOMDP(const Pair<number, number> &couple_agent) const;


//     protected:

//         std::vector<std::shared_ptr<MPOMDPInterface>> sub_pomdp;
//         std::vector<Pair<number, number>> list_neighboors;
        

//         number num_agents, horizon;

//         int current_timestep;

//         std::shared_ptr<State> internal_state;

//         double discount;

//         Criterion criterion;

//         std::shared_ptr<Space> state_space, action_space, observation_space;

//         std::shared_ptr<Distribution<std::shared_ptr<State>>> start_distribution;


//         std::map<int, int> transitionmatrix;

//         std::unordered_set<number> *stateSuccessor;

//         std::map<std::string, double> observationsmatrix;

//         std::map<std::string, std::unordered_set<number>> observationSuccessor;

//         /**
//          * @brief dynamics generator
//          */
//         std::unordered_map<std::string, std::discrete_distribution<number>> ndpomdp_dynamics_generator;

//         Node *n;

//         number *nodes;

//         number root;

//         std::ifstream input_file;

//         double rMax = 45;
//     };
// } // namespace sdm
