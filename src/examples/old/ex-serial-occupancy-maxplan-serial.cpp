#include <iostream>

#include <sdm/exception.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/core/state/occupancy_state.hpp>

#include<sdm/algorithms.hpp>
#include <sdm/algorithms/planning/hsvi.hpp>

using namespace sdm;

int main(int argc, char **argv)
{

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

		// Construct SerialOccupancyMDP using parser
		std::cout << "#> Parsing file \"" << filename << "\"\n";

		std::shared_ptr<SolvableByHSVI<TState, TAction>>  oMDP = std::make_shared<SerializedOccupancyMDP<TState, TAction>>(filename, horizon);

		// Creation of HSVI with maxplan 
		//auto p_algo =  sdm::algo::makeHSVI<TState, TAction>(oMDP, "", "maxplan_serial", "MaxInitializer", "MinInitialize", discount, error, horizon, trials, "Example-MaxPlan-OccupancyMDP");

		// other method to create HSVI with Maxplan
		// ***
		oMDP->getUnderlyingProblem()->setDiscount(discount);
		oMDP->getUnderlyingProblem()->setPlanningHorizon(horizon);

		if (oMDP->isSerialized())
		{
			horizon = horizon * oMDP->getUnderlyingProblem()->getNumAgents();
		}

		// Instanciate initializers
		auto lb_init = std::make_shared<MinInitializer<TState, TAction>>();
		auto ub_init = std::make_shared<MaxInitializer<TState, TAction>>();

		// Instanciate the max-plan serial representation of the lower bound
		//auto lower_bound = std::make_shared<MaxPlanValueFunction<TState, TAction>>(omdp_world, horizon, lb_init); //
		auto lower_bound = std::make_shared<sdm::MaxPlanValueFunctionSerialized<TState, TAction>>(oMDP, horizon, lb_init);

		// Instanciate the Tabular version for the upper bound
		auto upper_bound = std::make_shared<MappedValueFunction<TState, TAction>>(oMDP, horizon, ub_init);

		auto p_algo = std::make_shared<HSVI<TState, TAction>>(oMDP, lower_bound, upper_bound, horizon, error, trials, "Example-MaxPlan-OccupancyMDP");
		// *** 

		//Initialization of HSVI
		p_algo->initialize();

		//Resolution of HSVI
		p_algo->solve();
	}
	catch (exception::Exception &e)
	{
		std::cout << "!!! Exception: " << e.what() << std::endl;
	}

}