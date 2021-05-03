#include <iostream>

#include <sdm/exception.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/world/serialized_occupancy_mdp.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>
#include <sdm/world/serialized_occupancy_mdp.hpp>

#include <sdm/algorithms.hpp>

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


		using TState = SerializedOccupancyState<SerializedState, JointHistoryTree_p<number>>;
		using TAction = DeterministicDecisionRule<HistoryTree_p<number>, number>;

		auto serialized_oMDP = std::make_shared<SerializedOccupancyMDP<TState, TAction>>(filename, 3);

		auto p_algo = sdm::algo::makeHSVI<TState, TAction>(serialized_oMDP, "", "", "MaxInitializer", "MinInitializer", 1, 0, 3, 100, ("name" == "") ? "tab_ext_ohsvi" : "name");

		p_algo->do_initialize();

		std::cout<<"\n Lower_bound :"<<p_algo->getLowerBound();
		std::cout<<"\n Upper_bound :"<<p_algo->getUpperBound();

		p_algo->do_solve();	

		std::cout<<"\n Lower_bound :"<<p_algo->getLowerBound();
		std::cout<<"\n Upper_bound :"<<p_algo->getUpperBound();

		// using TState = SerializedOccupancyState<SerializedState, JointHistoryTree_p<number>>;
        // using TAction = DeterministicDecisionRule<HistoryTree_p<number>, number>;

		// // Construct Serial OccupancyMDP using parser
		// std::cout << "#> Parsing file \"" << filename << "\"\n";
		// auto somdp_world = std::make_shared<SerializedOccupancyMDP<TState, TAction>>(filename, 3);

		// // We will show how to expand an initial occupancy state and generate next ones using compression
		// int depth = 0, limit = 3;
		// auto ostate = somdp_world->getInitialState();
		// auto oaction = somdp_world->getActionSpaceAt(ostate)->sample();

		// std::cout << "#> Print depth \"" << depth << "\"\n";
		// std::cout << "#> Print occupancy state \n" << ostate << "\n";
		// std::cout << "#> Print joint decision rule \n" << oaction << "\n";

		// do
		// {
		// 	depth++;
		// 	std::cout << "#> Print depth \"" << depth << "\"\n";

		// 	// Compute the next compressed occupancy state
		// 	ostate = somdp_world->nextState(ostate, oaction);
		// 	std::cout << "#> Print compressed occupancy state \n" << ostate << "\n";
		// 	std::cout << "#> Print one step left occupancy state \n" << *ostate.getOneStepUncompressedOccupancy() << "\n";
		// 	std::cout << "#> Print fully uncompressed occupancy state \n" << *ostate.getFullyUncompressedOccupancy() << "\n";

		// 	// Sample a decision rule
		// 	oaction = somdp_world->getActionSpaceAt(ostate)->sample();
		// 	std::cout << "#> Print joint decision rule \n" << oaction << "\n";
		// } while (depth < limit);
	}
	catch (exception::Exception &e)
	{
		std::cout << "!!! Exception: " << e.what() << std::endl;
	}

	return 0;
} // END main