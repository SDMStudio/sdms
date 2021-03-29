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

#include <sdm/utils/value_function/initializer.hpp>

namespace sdm
{
    /**
     * @brief The MDP initializer enables to initialize the upperbound with the underlying MDP optimal value function.
     * 
     * @tparam TState the state type
     * @tparam TAction the action type
     * @param algo_name the algorithm that will be used to solve the underlying MDP.
     * @param error the maximal error
     * @param trials the maximal number of trials
     */
    template <typename TState, typename TAction>
    class MDPInitializer : public Initializer<TState, TAction>
    {
    public:
        std::string algo_name_;
        double error_, trials_;

    public:
        MDPInitializer(std::string algo_name, double error = 0.01, int trials = 10000);
        void init(ValueFunction<TState, TAction> *vf);
    };
} // namespace sdm
#include <sdm/utils/value_function/initializer/mdp_initializer.tpp>
