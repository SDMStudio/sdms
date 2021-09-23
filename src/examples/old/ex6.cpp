/**
 * @file ex6.cpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief Example of interacting with worlds (for reinforcement learning).
 * @version 1.0
 * @date 07/04/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <iostream>
#include <fstream>
#include <time.h>

#include <sdm/types.hpp>
#include <sdm/world/discrete_mdp.hpp>
#include <sdm/world/discrete_pomdp.hpp>
#include <sdm/world/discrete_decpomdp.hpp>
#include <sdm/world/belief_mdp.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    std::string filename;
    if (argc > 1)
    {
        filename = argv[1];
        std::cout << "#> Parsing file \"" << filename << "\"\n";
    }

    else
    {
        std::cerr << "Error: No input file provided." << std::endl;
        return 1;
    }

    try
    {

        number horizon = 5;
        number n_episode = 10;
        auto env = std::make_shared<BeliefMDP<>>(filename);
        // auto env = std::make_shared<DiscreteMDP>(filename);
        // auto env = std::make_shared<DiscretePOMDP>(filename);
        // auto env = std::make_shared<DiscreteDecPOMDP>(filename);
        env->getUnderlyingProblem()->setPlanningHorizon(horizon);

        for (number episode = 0; episode < n_episode; episode++)
        {
            env->reset();
            std::cout << "---------------" << std::endl;
            for (number step = 0; step < 1000; step++)
            {
                auto action = env->getUnderlyingProblem()->getActionSpace()->sample();
                auto [next_obs, rewards, done] = env->step(action);
                std::cout << "#Ep " << episode +1 << "/" << n_episode << " | Step " << step << " | act=" << action <<" obs=" << next_obs << " rewards=" << rewards << " done=" << done << std::endl;
                if (done)
                {
                    break;
                }
            }
        }
    }
    catch (sdm::exception::Exception &e)
    {
        std::cout << "!!! Exception: " << e.what() << std::endl;
    }
    return 0;
}