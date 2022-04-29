#include <iostream>

#include <sdm/config.hpp>
#include <sdm/exception.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/world/serial_mpomdp.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/world/serial_occupancy_mdp.hpp>
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
		int t = 0, horizon = 4, memory = 1;
		bool done = false;
		// Construct OccupancyMDP using parser
		std::cout << "#> Parsing file \"" << filename << "\"\n\n";
		auto mpomdp = parser::parse_file(filename);
		mpomdp->setHorizon(horizon);

		auto serial_mpomdp = std::make_shared<SerialMPOMDP>(mpomdp);
		std::shared_ptr<SolvableByDP> omdp_world = std::make_shared<SerialOccupancyMDP>(serial_mpomdp, memory);

		// We will show how to expand an initial occupancy state and generate next ones using compression
		auto ostate = omdp_world->getInitialState();

		std::cout << "\033[1;31m#> TIMESTEP = " << t << "\033[0m\n";
		std::cout << "\033[1;34m#> COMPRESSED INITIAL OCCUPANCY STATE \033[0m\n" << *ostate << "\n";
		do
		{
			std::cout << "\n\033[1;31m#> TIMESTEP = " << t + 1 << "\033[0m\n";

			auto oaction = omdp_world->getActionSpaceAt(ostate, t)->sample();
			std::cout << "\n\033[1;34m#> ACTION \033[0m\n" << *oaction << "\n\n";

			auto oobservation = omdp_world->getObservationSpaceAt(ostate, oaction, t)->sample();
			std::cout << "\n\033[1;34m#> OBSERVATION \033[0m\n" << *oobservation << "\n\n";

			// Compute the next compressed occupancy state;
			ostate = omdp_world->getNextStateAndProba(ostate, oaction, oobservation, t).first;
			std::cout << "\n\033[1;34m#> COMPRESSED OCCUPANCY STATE \033[0m\n"
					  << *ostate << "\n\n";
			std::cout << "\n\033[1;34m#> ONE STEP UNCOMPRESSED OCCUPANCY STATE \033[0m\n"
					  << *ostate->toOccupancyState()->getOneStepUncompressedOccupancy() << "\n\n";
			// std::cout << "#> Print fully uncompressed occupancy state \n" << *ostate->toOccupancyState()->getFullyUncompressedOccupancy() << "\n";

			t++;
		} while (t < omdp_world->getHorizon());
	}
	catch (exception::Exception &e)
	{
		std::cout << "!!! Exception: " << e.what() << std::endl;
	}

	return 0;
} // END main