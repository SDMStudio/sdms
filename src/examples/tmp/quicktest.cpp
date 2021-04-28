#include <iostream>

#include <sdm/parser/parser.hpp>
#include <sdm/exception.hpp>

#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>

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
		auto omdp_world = std::make_shared<sdm::OccupancyMDP<>>(filename);
		std::cout << "#> omdp_world reference \"" << omdp_world << "\"\n";

		// We will show how to expand an initial occupancy state and generate next ones
		int depth = 0, limit = 2;
		auto ostate = omdp_world->getInitialState();
		auto oaction = omdp_world->getActionSpaceAt(ostate)->sample();

		std::cout << "#> Print depth \"" << depth << "\"\n";
		std::cout << "#> Print occupancy state \n"
				  << ostate << "\n";
		for (sdm::number agent_id = 0; agent_id < 2; agent_id++)
		{
			for (const auto &pair_ihist_private_ostate : ostate.getPrivateOccupancyStates()[agent_id])
			{
				std::cout << "#> Private occupancy state : "
						  << " agent_id=" << agent_id
						  << " ihist=" << pair_ihist_private_ostate.first << std::endl
						  << *pair_ihist_private_ostate.second << std::endl;
			}
		}
		std::cout << "#> Print joint decision rule \n"
				  << oaction << "\n";

		do
		{
			depth++;
			std::cout << "#> Print depth \"" << depth << "\"\n";

			ostate = omdp_world->nextState(ostate, oaction);
			std::cout << "#> Print occupancy state \n"
					  << ostate << "\n";
			std::cout << "#> Private occupancy state \n"
					  << ostate.getPrivateOccupancyStates() << std::endl;
					  
			oaction = omdp_world->getActionSpaceAt(ostate)->sample();
			std::cout << "#> Print joint decision rule \n"
					  << oaction << "\n";
		} while (depth < limit);
	}
	catch (sdm::exception::Exception &e)
	{
		std::cout << "!!! Exception: " << e.what() << std::endl;
	}

	return 0;
} // END main