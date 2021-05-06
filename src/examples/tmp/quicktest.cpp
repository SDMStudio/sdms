#include <iostream>

#include <sdm/exception.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/world/serialized_occupancy_mdp.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>
#include <sdm/world/serialized_occupancy_mdp.hpp>

#include<sdm/algorithms.hpp>
#include <sdm/utils/value_function/sawtooth_vf_with_lp.hpp>


using namespace sdm;

int main(int argc, char **argv)
{
	std::string filename;
    number horizon = 1;
    number discount = 1;
    double error = 0.00001;
    number trials = 10;

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

		using TObservation = number;
		using TState = number;

		using TActionDescriptor = number;
		using TStateDescriptor = HistoryTree_p<TObservation>;

		using TActionPrescriptor = JointDeterministicDecisionRule<TStateDescriptor, TActionDescriptor>;
		using TStatePrescriptor = OccupancyState<TState, JointHistoryTree_p<TObservation>>;

		std::cout << "#> Parsing file \"" << filename << "\"\n";

		std::shared_ptr<SolvableByHSVI<TStatePrescriptor, TActionPrescriptor>>  oMDP = std::make_shared<OccupancyMDP<TStatePrescriptor, TActionPrescriptor>>(filename, horizon);        

		oMDP->getUnderlyingProblem()->setDiscount(discount);
		oMDP->getUnderlyingProblem()->setPlanningHorizon(horizon);

		// Instanciate initializers 
		auto lb_init = std::make_shared<MinInitializer<TStatePrescriptor, TActionPrescriptor>>();
		auto ub_init = sdm::makeInitializer<TStatePrescriptor, TActionPrescriptor>("MdpHsviInitializer");

		// Instanciate the Tabular version for the lower bound
		std::shared_ptr<sdm::ValueFunction<TStatePrescriptor, TActionPrescriptor>> lower_bound = std::make_shared<MappedValueFunction<TStatePrescriptor, TActionPrescriptor>>(oMDP, horizon, lb_init); 

		// Instanciate the Sawtooth version for the upper bound 
		std::shared_ptr<sdm::ValueFunction<TStatePrescriptor, TActionPrescriptor>> upper_bound = std::make_shared<SawtoothValueFunctionLP<TStatePrescriptor, TActionPrescriptor>>(oMDP, horizon, ub_init);

		auto p_algo = std::make_shared<HSVI<TStatePrescriptor, TActionPrescriptor>>(oMDP, lower_bound, upper_bound, horizon, error, trials, "Example-MaxPlan-OccupancyMDP");

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