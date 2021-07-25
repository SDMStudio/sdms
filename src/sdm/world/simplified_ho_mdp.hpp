// #pragma once

// #include <sdm/types.hpp>
// #include <sdm/core/state/state.hpp>
// #include <sdm/core/state/occupancy_state.hpp>
// #include <sdm/core/state/interface/occupancy_state_interface.hpp>
// #include <sdm/core/action/action.hpp>
// #include <sdm/utils/struct/recursive_map.hpp>
// #include <sdm/world/base/pomdp_interface.hpp>
// #include <sdm/world/gym_interface.hpp>
// #include <sdm/world/mdp.hpp>

// namespace sdm
// {
//     class SimplifiedHOMDP : public GymInterface
//     {
//     public:
//         SimplifiedHOMDP();
//         SimplifiedHOMDP(const std::shared_ptr<POMDPInterface> &pomdp, int batch_size = 0);

//         /**
//          * @brief Get the next belief.
//          * This function returns the next belief. To do so, we check in the MDP graph the existance of an edge (action / observation) starting from the current belief. 
//          * If exists, we return the associated next belief. Otherwise, we compute the next belief using  "computeNextStateAndProba" function and add the edge from the current belief to the next belief in the graph.
//          * 
//          * @param belief the belief
//          * @param action the action
//          * @param observation the observation
//          * @param t the timestep
//          * @return the next belief
//          */
//         std::shared_ptr<State> nextBelief(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);

//         /** @brief Get the address of the underlying POMDP */
//         std::shared_ptr<POMDPInterface> getUnderlyingPOMDP() const;

//         /**
//          * @brief Get the action space at a specific belief and timestep.
//          * The time dependency is required in extensive-form games in which some agents have a different action space.   
//          * 
//          * @param belief the belief
//          * @param t the timestep
//          * @return the action space 
//          */
//         std::shared_ptr<Space> getActionSpaceAt(const std::shared_ptr<State> &belief, number t = 0);

//         /**
//          * @brief Get the expected reward of executing a specific action in a specific belief at timestep t. 
//          * The time dependency can be required in non-stationnary problems.   
//          * 
//          * @param belief the belief
//          * @param action the action
//          * @param t the timestep
//          * @return the reward
//          */
//         double getReward(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t = 0);

//         std::shared_ptr<Observation> reset();
//         std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> step(std::shared_ptr<Action> action);
//         std::shared_ptr<Space> getActionSpaceAt(const std::shared_ptr<Observation> &observation, number t);
//         std::shared_ptr<Action> getRandomAction(const std::shared_ptr<Observation> &observation, number t);

//     protected:
//         // If 0, it means the exact transitions will be used and not sampled ones.
//         int batch_size_;

//         /** @brief The current state (used in RL). */
//         std::shared_ptr<OccupancyStateInterface> initial_state_, current_state_;

//         /** @brief The current timestep (used in RL). */
//         int step_;
        
//         Pair<std::shared_ptr<OccupancyStateInterface>, double> computeNextStateAndProbability(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);
//         std::shared_ptr<OccupancyStateInterface> computeNextState(const std::shared_ptr<OccupancyStateInterface> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);
//     };

// }
// #include <sdm/world/belief_mdp.tpp>
