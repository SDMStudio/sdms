#include <iostream>

#include <sdm/exception.hpp>
#include <sdm/core/joint.hpp>
#include <sdm/core/action/hierarchical_private_joint_det_decision_rule.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>
#include <sdm/world/hierarchical_private_occupancy_mdp.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
	std::string filename;
	int limit;

	if (argc == 2)
	{
		filename = argv[1];
		limit = 5;
	} 
	
	else if (argc == 3)
	{
		filename = argv[1];
		limit = atoi(argv[2]);
	}

	else
	{
		std::cerr << "Error: Require 1 input file." << std::endl;
		return 1;
	}

	try
	{
		using TState = OccupancyState<number, JointHistoryTree_p<number>>;
		using TAction = HierarchicalPrivateJointDeterministicDecisionRule<Joint<HistoryTree_p<number>>, number>;
		// using TAction = PrivateHierarchicalJointDeterministicDecisionRule<HistoryTree_p<number>, number>;

		// Construct hpoMDP using parser
		std::cout << "#> Parsing file \"" << filename << "\"\n";
		auto hpoMDP = std::make_shared<HierarchicalPrivateOccupancyMDP<TState, TAction>>(filename);
		// We will show how to expand an initial private occupancy state and generate next ones using compression
		int depth = 0;
		std::cout << "#> Depth: " << depth << "\n";
		auto postate = hpoMDP->getInitialState();
		std::cout << "#> Private occupancy state: \n"
				  		<< postate << "\n";
		// std::cout << "#> Fully uncompressed occupancy state \n"
		// 					<< *postate.getFullyUncompressedOccupancy() << "\n";
		std::cout << "Uncompressed size: " << postate.getFullyUncompressedOccupancy()->getSize() << "\n";
		std::cout << "Compressed size: " << postate.getSize() << "\n";
		auto oaction_space = hpoMDP->getActionSpaceAt(postate);
		// std::cout << "#> All possible oactions: \n";
		// for (auto oaction: oaction_space->getAll()){
		// 	std::cout << oaction;
		// }
		auto oaction = oaction_space->sample();
		std::cout << "#> Private hierarchical joint decision rule: \n"
				  		<< oaction << "\n";

		do
		{
			std::cout << "\n" << "#> Depth: " << ++depth << "\n";

			// Compute the next compressed occupancy state
			postate = hpoMDP->nextState(postate, oaction, 0);
			std::cout << "#> Print compressed occupancy state \n"
					  		<< postate << "\n";
			std::cout << "Uncompressed size: " << postate.getFullyUncompressedOccupancy()->getSize() << "\n";
			std::cout << "Compressed size: " << postate.getSize() << "\n";
			// std::cout << "#> Print fully uncompressed occupancy state \n"
			// 		  		<< *postate.getFullyUncompressedOccupancy() << "\n";

			// Sample a decision rule
			oaction_space = hpoMDP->getActionSpaceAt(postate);
			std::cout << "#> Size of oaction_space: " << oaction_space->getNumItems() << "\n";
			// std::cout << "#> All possible oactions: \n";
			// for (auto oaction: oaction_space->getAll()){
			// 	std::cout << oaction;
			// }
			oaction = oaction_space->sample();
			std::cout << "#> Oaction: \n"
					  		<< oaction << "\n";
		} while (depth < limit);
	}
	catch (exception::Exception &e)
	{
		std::cout << "!!! Exception: " << e.what() << std::endl;
	}

	return 0;
} // END main
