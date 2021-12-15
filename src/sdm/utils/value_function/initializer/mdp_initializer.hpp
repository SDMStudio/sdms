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

#include <sdm/utils/config.hpp>
#include <sdm/utils/value_function/initializer/initializer.hpp>

namespace sdm
{
    /**
     * @brief The MDP initializer enables to initialize the upper bound in HSVI with the underlying MDP optimal value function.
     * This is a common usage in HSVI to use the solution of a relaxation of the problem in order to get a accurate upper bound (see also the class POMDPInitializer ).
     *
     * @tparam TState the state type
     * @tparam TAction the action type
     */
    class MDPInitializer : public Initializer
    {
    public:
        Config algo_config;
        std::shared_ptr<SolvableByDP> world;

    public:
        MDPInitializer(std::shared_ptr<SolvableByDP> world, Config config);
        MDPInitializer(std::shared_ptr<SolvableByDP> world, std::string algo_name, double error = 0.0001, int trials = 20000);
        void init(std::shared_ptr<ValueFunctionInterface> vf);
    };
} // namespace sdm
