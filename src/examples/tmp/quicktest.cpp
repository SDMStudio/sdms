#include <iostream>

#include <sdm/exception.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/state/serialized_occupancy_state.hpp>
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
		// Construct OccupancyMDP using parser
		std::cout << "#> Parsing file \"" << filename << "\"\n";

		auto omdp_world = std::make_shared<SerializedOccupancyMDP<>>(filename);
		SerializedOccupancyState();

		// // We will show how to expand an initial occupancy state and generate next ones using compression
		// int depth = 0, limit = 3;
		// auto ostate = omdp_world->getInitialState();
		// auto oaction = omdp_world->getActionSpaceAt(ostate)->sample();

		// std::cout << "#> Print depth \"" << depth << "\"\n";
		// std::cout << "#> Print occupancy state \n" << ostate << "\n";
		// std::cout << "#> Print joint decision rule \n" << oaction << "\n";

		// do
		// {
		// 	depth++;
		// 	std::cout << "#> Print depth \"" << depth << "\"\n";

        //     // Compute the next compressed occupancy state
		// 	ostate = omdp_world->nextState(ostate, oaction);
		// 	std::cout << "#> Print compressed occupancy state \n" << ostate << "\n";
		// 	std::cout << "#> Print one step left occupancy state \n" << *ostate.getOneStepUncompressedOccupancy() << "\n";
		// 	std::cout << "#> Print fully uncompressed occupancy state \n" << *ostate.getFullyUncompressedOccupancy() << "\n";

        //     // Sample a decision rule
		// 	oaction = omdp_world->getActionSpaceAt(ostate)->sample();
		// 	std::cout << "#> Print joint decision rule \n"
		// 			  << oaction << "\n";
		// } while (depth < limit);
	}
	catch (exception::Exception &e)
	{
		std::cout << "!!! Exception: " << e.what() << std::endl;
	}

	return 0;
} // END main