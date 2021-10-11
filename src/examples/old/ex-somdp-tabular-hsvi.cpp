#include <iostream>

#include <sdm/exception.hpp>
#include <sdm/world/serial_occupancy_mdp.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>
#include <sdm/world/serial_occupancy_mdp.hpp>

#include<sdm/algorithms.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    // ************** Exemple Next State in Serial Occupancy MDP

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
		using TState = SerialOccupancyState<SerialState, JointHistoryTree_p<number>>;
        using TAction = DeterministicDecisionRule<HistoryTree_p<number>, number>;

		// Construct Serial OccupancyMDP using parser
		std::cout << "#> Parsing file \"" << filename << "\"\n";
		auto somdp_world = std::make_shared<SerialOccupancyMDP<TState, TAction>>(filename, horizon);

        // // ************ Exemple HSVI for Serial Occupancy MDP

        // //HSVI with default parameter
        // auto p_algo = sdm::algo::makeHSVI<TState, TAction>(somdp_world, "", "", "MaxInitializer", "MinInitializer", discount, error, horizon, trials, "Exemple_SerialOccupancyMDP_HSVI");

        // p_algo->initialize();
        // p_algo->solve();

	}
	catch (exception::Exception &e)
	{
		std::cout << "!!! Exception: " << e.what() << std::endl;
	}
}