/**
 * @file ex2.cpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief File that give some exemples of operators that can be used on formalisms
 * @version 1.0
 * @date 01/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <iostream>
#include <cassert>
#include <sdm/worlds.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/exception.hpp>

int main(int argc, char **argv)
{
	char const *filename;

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
		auto dpomdp_world = sdm::parser::parse_file(filename);

		std::cout << "#> XML\n\n";
		std::cout << dpomdp_world->toXML() << "\n\n";

		std::cout << "#> Standard Format\n\n";
		std::cout << dpomdp_world->toStdFormat() << "\n\n";

		std::cout << "#> Filename : " << dpomdp_world->getFileName() << "\n\n";
		std::cout << "#> Criterion : " << dpomdp_world->getCriterion() << "\n\n";
		std::cout << "#> Discount : " << dpomdp_world->getDiscount() << "\n\n";

		std::cout << "#> STATE SPACE \n\n";
		std::cout << "n_states=" << dpomdp_world->getStateSpace()->getNumItems() << "\n\n";
		std::cout << "state_space=" << *dpomdp_world->getStateSpace() << "\n\n";

		std::cout << "#> AGENT SPACE \n\n";
		std::cout << "n_agents=" << dpomdp_world->getNumAgents() << "\n\n";

		std::cout << "#> Action SPACE \n\n";
		std::cout << "n_joint_actions=" << dpomdp_world->getActionSpace()->getNumItems() << "\n\n";
		std::cout << "action_space=" << *dpomdp_world->getActionSpace() << "\n\n";

		std::cout << "#> Observation SPACE \n\n";
		std::cout << "n_joint_observations=" << dpomdp_world->getObsSpace()->getNumItems() << "\n\n";
		std::cout << "observation_space=" << *dpomdp_world->getObsSpace() << "\n\n";

		std::cout << "#> Reward function : \n\n"
				  << *dpomdp_world->getRewardSpace() << "\n\n";
	}
	catch (sdm::exception::Exception &e)
	{
		std::cout << "!!! Exception: " << e.what() << std::endl;
	}
	return 0;
}
