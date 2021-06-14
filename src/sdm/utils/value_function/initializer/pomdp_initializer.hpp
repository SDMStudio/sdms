/**
 * @file mdp_initializer.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief The file that contains MDPInitializer class
 * @version 1.0
 * @date 29/03/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/utils/value_function/initializer/initializer.hpp>

namespace sdm
{
    /**
     * @brief The POMDP initializer enables to initialize the upper bound in HSVI with the underlying POMDP optimal value function.
     * 
     * @tparam TState the state type
     * @tparam TAction the action type
     * @param algo_name the algorithm that will be used to solve the underlying POMDP.
     * @param error the maximal error
     * @param trials the maximal number of trials
     */
    template <typename TState, typename TAction>
    class POMDPInitializer : public Initializer<TState, TAction>
    {
    public:
        std::string algo_name_;
        double error_, trials_;

    public:
        POMDPInitializer(std::string algo_name, double error = 0.01, int trials = 10000);
        void init(std::shared_ptr<ValueFunction<TState, TAction>> vf);
    };
} // namespace sdm
#include <sdm/utils/value_function/initializer/pomdp_initializer.tpp>
