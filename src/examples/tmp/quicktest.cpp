#include <iostream>

#include <sdm/parser/parser.hpp>
#include <sdm/exception.hpp>

#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/world/discrete_decpomdp.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
	std::string filename;

	if (argc > 1)
	{
		filename = argv[1];
	}

	else
	{
		std::cerr << "Error: Require 1 input file." << std::endl;
		return 1;
	}

	try
	{
		// Construct DecPOMDP using parser
		std::cout << "#> Parsing file \"" << filename << "\"\n";
		// auto omdp_world = sdm::parser::parse_file(filename);
		// auto world = std::make_shared<DiscreteDecPOMDP>(filename);
		auto omdp_world = std::make_shared<OccupancyMDP<>>(filename);
		std::cout << "#> omdp_world reference \"" << omdp_world << "\"\n";

		// Test getReachable
		// world->setupDynamicsGenerator();
		// auto state_sample = world->getStateSpace()->sample();
		// std::cout << "state_sample=" << state_sample << std::endl;

		// auto action_sample = world->getActionSpace()->sample();
		// std::cout << "action_sample=" << action_sample << std::endl;

		// for (auto state : world->getReachableStates(state_sample, action_sample))
		// {
		// 	std::cout << "S : " << state << std::endl;
		// }

		// for (auto obs : world->getReachableObservations(action_sample, state_sample))
		// {
		// 	std::cout << "O : " << obs << std::endl;
		// }

		// We will show how to expand an initial occupancy state and generate next ones
		int depth = 0, limit = 3;
		auto ostate = omdp_world->getInitialState();
		auto oaction = omdp_world->getActionSpaceAt(ostate)->sample();

		std::cout << "#> Print depth \"" << depth << "\"\n";
		std::cout << "#> Print occupancy state \n"
				  << ostate << "\n";
		std::cout << "#> Print joint decision rule \n"
				  << oaction << "\n";

		do
		{
			depth++;
			std::cout << "#> Print depth \"" << depth << "\"\n";

			ostate = omdp_world->nextState(ostate, oaction);
			std::cout << "#> Print occupancy state \n"
					  << ostate << "\n";

			oaction = omdp_world->getActionSpaceAt(ostate)->sample();
			std::cout << "#> Print joint decision rule \n"
					  << oaction << "\n";
		} while (depth < limit);
	}
	catch (exception::Exception &e)
	{
		std::cout << "!!! Exception: " << e.what() << std::endl;
	}

	return 0;
} // END main