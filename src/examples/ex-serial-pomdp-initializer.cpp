#include <iostream>

#include <sdm/exception.hpp>
#include <sdm/algorithms.hpp>
#include <sdm/world/serialized_belief_mdp.hpp>
#include <sdm/core/state/serialized_belief_state.hpp>

#include <sdm/utils/value_function/tabular_value_function.hpp>
#include <sdm/utils/value_function/initializers.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
	std::string filename;
    number horizon;
    number discount = 1;
    double error = 0.00001;
    number trials = 100;

	if (argc > 2)
	{
		filename = argv[1];
		horizon = std::atoi( argv[2] );
	}

	else
	{
		std::cerr << "Error: Require 1 input file." << std::endl;
		return 1;
	}

	try
	{
		using TStatePrescriptor = SerializedOccupancyState<SerializedState, JointHistoryTree_p<number>>;
		using TActionPrescriptor = DeterministicDecisionRule<HistoryTree_p<number>, number>;

		std::cout << "#> Parsing file \"" << filename << "\"\n";

		std::shared_ptr<SolvableByHSVI<TStatePrescriptor, TActionPrescriptor>> serialized_oMDP = std::make_shared<SerializedOccupancyMDP<TStatePrescriptor, TActionPrescriptor>>(filename, horizon);;        

		serialized_oMDP->getUnderlyingProblem()->setDiscount(discount);
		serialized_oMDP->getUnderlyingProblem()->setPlanningHorizon(horizon);

		if (serialized_oMDP->isSerialized())
		{
			horizon = horizon * serialized_oMDP->getUnderlyingProblem()->getNumAgents();
		}

		// Instanciate initializers 
		auto lb_init = std::make_shared<MinInitializer<TStatePrescriptor, TActionPrescriptor>>();
		auto ub_init = std::make_shared<POMDPInitializer<TStatePrescriptor, TActionPrescriptor>>("PomdpInitializer");

		// Instanciate the Tabular version for the lower bound
		std::shared_ptr<sdm::ValueFunction<TStatePrescriptor, TActionPrescriptor>> lower_bound = std::make_shared<MappedValueFunction<TStatePrescriptor, TActionPrescriptor>>(serialized_oMDP, horizon, lb_init); 

		// Instanciate the Tabular version for the upper bound 
		std::shared_ptr<sdm::ValueFunction<TStatePrescriptor, TActionPrescriptor>> upper_bound = std::make_shared<MappedValueFunction<TStatePrescriptor, TActionPrescriptor>>(serialized_oMDP, horizon, ub_init);

		auto p_algo = std::make_shared<HSVI<TStatePrescriptor, TActionPrescriptor>>(serialized_oMDP, lower_bound, upper_bound, horizon, error, trials, "Example-Occupancy-MDP-POMDPInitializer");

		//Initialization of HSVI
		p_algo->do_initialize();

		//Resolution of HSVI
		p_algo->do_solve();
	}
	catch (exception::Exception &e)
	{
		std::cout << "!!! Exception: " << e.what() << std::endl;
	}

	return 0;
} // END main