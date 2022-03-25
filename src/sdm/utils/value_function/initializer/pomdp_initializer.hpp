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

#include <sdm/utils/config.hpp>
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
    class POMDPInitializer : public Initializer
    {
    public:
        Config algo_config;
        std::shared_ptr<SolvableByDP> world;

    public:
        POMDPInitializer(std::shared_ptr<SolvableByDP> world, Config config);
        POMDPInitializer(std::shared_ptr<SolvableByDP> world, std::string algo_name, double error = 0.001, int trials = 20000);
        void init(std::shared_ptr<ValueFunctionInterface> vf);
    };
} // namespace sdm
