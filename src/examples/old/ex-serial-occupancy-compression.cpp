#include <iostream>

#include <sdm/exception.hpp>
#include <sdm/world/serialized_occupancy_mdp.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>
#include <sdm/world/serialized_occupancy_mdp.hpp>

#include<sdm/algorithms.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    // ************** Exemple Next State in Serialized Occupancy MDP

	std::string filename;
    number horizon = 3;
    number discount = 1;
    double error = 0.00001;
    number trials = 1000;

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
		using TState = SerializedOccupancyState<SerializedState, JointHistoryTree_p<number>>;
        using TAction = DeterministicDecisionRule<HistoryTree_p<number>, number>;

		// Construct Serial OccupancyMDP using parser
		std::cout << "#> Parsing file \"" << filename << "\"\n";
		auto somdp_world = std::make_shared<SerializedOccupancyMDP<TState, TAction>>(filename, horizon);

		// We will show how to expand an initial occupancy state and generate next ones using compression
		int depth = 0, limit = 3;
		auto ostate = somdp_world->getInitialState();
		auto oaction = somdp_world->getActionSpaceAt(ostate)->sample();

		std::cout << "#> Print depth \"" << depth << "\"\n";
		std::cout << "#> Print occupancy state \n" << ostate << "\n";
		std::cout << "#> Print joint decision rule \n" << oaction << "\n";

		do
		{
		 	depth++;
		 	std::cout << "#> Print depth \"" << depth << "\"\n";

			// Compute the next compressed occupancy state
			ostate = somdp_world->nextState(ostate, oaction);
			std::cout << "#> Print compressed occupancy state \n" << ostate << "\n";
			std::cout << "#> Print one step left occupancy state \n" << *ostate.getOneStepUncompressedOccupancy() << "\n";
			std::cout << "#> Print fully uncompressed occupancy state \n" << *ostate.getFullyUncompressedOccupancy() << "\n";

			// Sample a decision rule
			oaction = somdp_world->getActionSpaceAt(ostate)->sample();
			std::cout << "#> Print joint decision rule \n" << oaction << "\n";
		} while (depth < limit);

	}
	catch (exception::Exception &e)
	{
		std::cout << "!!! Exception: " << e.what() << std::endl;
	}
}