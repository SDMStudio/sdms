#include <iostream>

#include <sdm/exception.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/world/occupancy_mdp.hpp>
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
		// Construct OccupancyMDP using parser
		std::cout << "#> Parsing file \"" << filename << "\"\n";
		auto mpomdp = parser::parse_file(filename);
		auto omdp_world = std::make_shared<OccupancyMDP>(mpomdp);

		// We will show how to expand an initial occupancy state and generate next ones using compression
		int depth = 0, limit = 3;
		auto ostate = omdp_world->getInitialState();
		auto oaction = omdp_world->getActionSpaceAt(ostate)->sample()->toAction();

		std::cout << "#> Print depth \"" << depth << "\"\n";
		std::cout << "#> Print occupancy state \n" << ostate << "\n";
		std::cout << "#> Print joint decision rule \n" << oaction << "\n";

		do
		{
			std::cout << "#> Print depth \"" << depth + 1 << "\"\n";

            // Compute the next compressed occupancy state
			ostate = omdp_world->getNextStateAndProba(ostate, oaction, NO_OBSERVATION, depth).first;
			std::cout << "#> Print compressed occupancy state \n" << *ostate << "\n";
			std::cout << "#> Print one step left occupancy state \n" << *ostate->toOccupancyState()->getOneStepUncompressedOccupancy() << "\n";
			std::cout << "#> Print fully uncompressed occupancy state \n" << *ostate->toOccupancyState()->getFullyUncompressedOccupancy() << "\n";

            // Sample a decision rule
			oaction = omdp_world->getActionSpaceAt(ostate)->sample()->toAction();
			std::cout << "#> Print joint decision rule \n" << oaction << "\n";
			depth++;
		} while (depth < limit);
	}
	catch (exception::Exception &e)
	{
		std::cout << "!!! Exception: " << e.what() << std::endl;
	}

	return 0;
} // END main