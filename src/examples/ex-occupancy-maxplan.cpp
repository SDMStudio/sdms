#include <iostream>

#include <sdm/exception.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/core/state/occupancy_state.hpp>

#include<sdm/algorithms.hpp>
#include <sdm/algorithms/hsvi.hpp>

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

		using TObservation = number;
		using TState = number;

		using TActionDescriptor = number;
		using TStateDescriptor = HistoryTree_p<TObservation>;

		// using TActionPrescriptor = Joint<DeterministicDecisionRule<TStateDescriptor, TActionDescriptor>>;
		using TActionPrescriptor = JointDeterministicDecisionRule<TStateDescriptor, TActionDescriptor>;
		using TStatePrescriptor = OccupancyState<TState, JointHistoryTree_p<TObservation>>;

		// Construct OccupancyMDP using parser
		std::cout << "#> Parsing file \"" << filename << "\"\n";

		std::shared_ptr<SolvableByHSVI<TStatePrescriptor, TActionPrescriptor>>  oMDP = std::make_shared<OccupancyMDP<TStatePrescriptor,TActionPrescriptor>>(filename, horizon);


		// Creation of HSVI with maxplan 
		//auto p_algo =  sdm::algo::makeHSVI<TStatePrescriptor, TActionPrescriptor>(oMDP, "", "maxplan", "MaxInitializer", "MinInitialize", discount, error, horizon, trials, "Example-MaxPlan-OccupancyMDP");

		// other method to create HSVI with Maxplan
		// ***
		oMDP->getUnderlyingProblem()->setDiscount(discount);
		oMDP->getUnderlyingProblem()->setPlanningHorizon(horizon);

		// Instanciate initializers
		auto lb_init = std::make_shared<MinInitializer<TStatePrescriptor, TActionPrescriptor>>();
		auto ub_init = std::make_shared<MaxInitializer<TStatePrescriptor, TActionPrescriptor>>();

		// Instanciate the max-plan representation of the lower bound
		//auto lower_bound = std::make_shared<MaxPlanValueFunction<TState, TAction>>(omdp_world, horizon, lb_init); //
		auto lower_bound = std::make_shared<sdm::MaxPlanValueFunction<TStatePrescriptor, TActionPrescriptor>>(oMDP, horizon, lb_init);

		// Instanciate the Tabular version for the upper bound
		auto upper_bound = std::make_shared<MappedValueFunction<TStatePrescriptor, TActionPrescriptor>>(oMDP, horizon, ub_init);

		auto p_algo = std::make_shared<HSVI<TStatePrescriptor, TActionPrescriptor>>(oMDP, lower_bound, upper_bound, horizon, error, trials, "Example-MaxPlan-OccupancyMDP");
		// *** 

		//Initialization of HSVI
		p_algo->do_initialize();

		//Resolution of HSVI
		p_algo->do_solve();
	}
	catch (exception::Exception &e)
	{
		std::cout << "!!! Exception: " << e.what() << std::endl;
	}

}