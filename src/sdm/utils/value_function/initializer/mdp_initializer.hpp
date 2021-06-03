/**
 * @file mdp_initializer.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief The file that contains the class for initialization base on MDP.
 * @version 1.0
 * @date 29/03/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/algorithms/hsvi.hpp>
#include <sdm/algorithms/value_iteration.hpp>
#include <sdm/utils/value_function/initializer.hpp>

namespace sdm
{
    /**
     * @brief The MDP initializer enables to initialize the upper bound in HSVI with the underlying MDP optimal value function. 
     * This is a common usage in HSVI to use the solution of a relaxation of the problem in order to get a accurate upper bound (see also the class POMDPInitializer ). 
     * 
     * @tparam TState the state type
     * @tparam TAction the action type
     */
    template <typename TState, typename TAction>
    class MDPInitializer : public Initializer
    {
    public:
        std::string algo_name_;
        double error_, trials_;

    public:
        MDPInitializer(std::string algo_name, double error = 0.01, int trials = 10000);
        void init(std::shared_ptr<ValueFunction> vf);
    };
} // namespace sdm
#include <sdm/utils/value_function/initializer/mdp_initializer.tpp>
