/**
 * @file ex5.cpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief File example for interacting with an environment in RL. 
 * @version 1.0
 * @date 17/03/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <iostream>
#include <sdm/common.hpp>
#include <sdm/worlds.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
	std::string filename;

	if (argc > 1)
	{
		filename = argv[1];
		std::cout << "#> Parsing NDPOMDP file \"" << filename << "\"\n";
	}

	else
	{
		std::cerr << "Error: No input file provided." << std::endl;
		return 1;
	}

	// auto mdp = std::make_shared<DiscreteMDP>(filename);
	// auto mdp = std::make_shared<DiscretePOMDP>(filename);
	// auto mdp = std::make_shared<DiscreteMMDP>(filename);
	auto mdp = std::make_shared<DiscreteDecPOMDP>(filename);

	mdp->setPlanningHorizon(5);
	mdp->setupDynamicsGenerator();

	for (number ep = 0; ep < 10; ep++)
	{

		std::vector<double> rewards;
		bool is_done = false;
		mdp->reset();
		// number obs = mdp->reset();
		// std::cout << "Observation initial : " << obs << std::endl;

		while (!is_done)
		{
			auto random_action = mdp->getActionSpace()->sample();
			std::cout << "Execute Action : " << random_action << std::endl;
			auto feedback = mdp->step(random_action);
			auto obs2 = std::get<0>(feedback);
			rewards = std::get<1>(feedback);
			is_done = std::get<2>(feedback);
			std::cout << "( " << obs2 << ", " << rewards[0] << ", " << is_done << " )" << std::endl;
		}
	}

	return 0;
}