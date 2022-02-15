// #pragma once

// #include <sdm/world/occupancy_mdp.hpp>

// namespace sdm
// {
//     /**
//      * @brief 
//      * 
//      */
//     class NDOccupancyMDP : public BaseOccupancyMDP<JointBelief>
//     {
//     public:
//         NDOccupancyMDP();
//         NDOccupancyMDP(const std::shared_ptr<NDPOMDPInterface> &ndpomdp, int memory = -1, bool compression = true, bool store_states = true, bool store_actions = true, int batch_size = 0);


//         Pair<std::shared_ptr<State>, std::shared_ptr<State>> computeExactNextState(const std::shared_ptr<State> &occupancy_state,
//                                                                                    const std::shared_ptr<Action> &decision_rule,
//                                                                                    const std::shared_ptr<Observation> &observation,
//                                                                                    number t = 0);


//         double getReward(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t = 0);

//     protected:
//     };
// } // namespace sdm
