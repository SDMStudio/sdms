// /**
//  * @file discrete_mdp.hpp
//  * @author David Albert (david.albert@insa-lyon.fr)
//  * @brief The file that contains the MDP class.
//  * @version 1.0
//  * @date 02/02/2021
//  * 
//  * @copyright Copyright (c) 2021
//  * 
//  */
// #pragma once

// #include <sdm/types.hpp>
// #include <sdm/world/belief_mdp.hpp>
// #include <sdm/world/discrete_pomdp.hpp>
// #include <sdm/world/discrete_decpomdp.hpp>
// #include <sdm/world/decision_process.hpp>
// #include <sdm/core/space/discrete_space.hpp>
// #include <sdm/core/state_dynamics.hpp>
// #include <sdm/core/reward.hpp>

// namespace sdm
// {
//     /**
//      * @brief The class for Discrete Markov Decision Processes. 
//      * 
//      */
//     template <typename TState, typename TAction>
//     class MDP : public FullyObservableDecisionProcess<DiscreteSpace<TState>, DiscreteSpace<TAction>, StateDynamics, Reward, std::discrete_distribution<number>>,
//                 public SolvableByHSVI<TState, TAction>,
//                 public std::enable_shared_from_this<MDP<TState, TAction>>
//     {
//     public:
//         MDP();

//         MDP(std::shared_ptr<DiscreteSpace<TState>> state_space, std::shared_ptr<DiscreteSpace<TAction>> action_space);

//         MDP(std::shared_ptr<DiscreteSpace<TState>> state_space, std::shared_ptr<DiscreteSpace<TAction>> action_space, std::discrete_distribution<number>);

//         MDP(std::shared_ptr<DiscreteSpace<TState>> state_space,
//             std::shared_ptr<DiscreteSpace<TAction>> action_space,
//             std::shared_ptr<StateDynamics> state_dynamics,
//             std::shared_ptr<Reward> reward_function,
//             std::discrete_distribution<number> initial_distribution,
//             number horizon = 0,
//             double discount = 0.9,
//             Criterion criterion = Criterion::REW_MAX);

//         MDP(std::string &filename);

//         // SolvableByHSVI interface implementation
//         TState getInitialState();

//         TState nextState(const TState &state, const TAction &action, number t = 0, std::shared_ptr<HSVI<TState, TAction>> hsvi = nullptr) const;

//         std::shared_ptr<DiscreteSpace<TAction>> getActionSpaceAt(const TState &state);

//         std::shared_ptr<Reward> getReward() const;

//         double getReward(const TState &state, const TAction &action) const;

//         double getExpectedNextValue(std::shared_ptr<ValueFunction<TState, TAction>> value_function, const TState &state, const TAction &action, number t = 0) const;

//         DiscreteMDP *getUnderlyingProblem();

//         bool isSerialized() const;

//         /**
//          * @brief Get the specific discount factor for the problem at hand
//          * 
//          * @param number decision epoch or any other parameter 
//          * @return double discount factor
//          */
//         double getDiscount(number = 0);

//         /**
//          * @brief Get the specific weighted discount factor for the problem at hand
//          * 
//          * @param number decision epoch or any other parameter 
//          * @return double discount factor
//          */
//         double getWeightedDiscount(number);

//         /**
//          * @brief  Compute the excess of the HSVI paper. It refers to the termination condition.
//          * 
//          * @param double : incumbent 
//          * @param double : lb value
//          * @param double : ub value
//          * @param double : cost_so_far 
//          * @param double : error 
//          * @param number : horizon 
//          * @return double 
//          */
//         double do_excess(double, double, double, double, double, number);

//         /**
//          * @brief  Select the next action
//          * 
//          * @param const std::shared_ptr<ValueFunction<TState, TAction>>& : the lower bound
//          * @param const std::shared_ptr<ValueFunction<TState, TAction>>& : the upper bound
//          * @param const TState & s : current state
//          * @param number h : horizon
//          * @return TAction 
//          */
//         number selectNextAction(const std::shared_ptr<ValueFunction<TState, TAction>> &lb, const std::shared_ptr<ValueFunction<TState, TAction>> &ub, const TState &state, number t);

//         std::shared_ptr<MDP<TState, TAction>> toMDP();

//         std::shared_ptr<MDP<TState, TAction>> getptr();
//     };
// } // namespace sdm

// #include <sdm/world/base/base_mdp.tpp>