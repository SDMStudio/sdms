#include <iostream>

#include <memory>
#include <sdm/exception.hpp>
// #include <sdm/core/state/beliefs.hpp>
// #include <sdm/world/serial_belief_mdp.hpp>
// #include <sdm/world/discrete_pomdp.hpp>
#include <sdm/world/discrete_mdp.hpp>
// #include <sdm/world/occupancy_mdp.hpp>
// #include <sdm/world/serial_occupancy_mdp.hpp>
// #include <sdm/core/state/occupancy_state.hpp>
// #include <sdm/core/state/serial_occupancy_state.hpp>
// #include <sdm/core/action/joint_det_decision_rule.hpp>
#include <sdm/utils/value_function/initializer/initializer.hpp>
// #include <sdm/utils/value_function/max_plan_vf.hpp>
// #include <sdm/utils/value_function/tabular_value_function.hpp>
// #include <sdm/algorithms/planning/hsvi.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
	std::string filename;
	number horizon = 4;
	double discount = 1.0, error = 0.01, trial = 1000;

	if (argc > 1)
	{
		filename = argv[1];

		if (argc > 2)
		{
			horizon = std::stoi(argv[2]);
		}
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

		// using TActionDescriptor = number;
		// using TStateDescriptor = HistoryTree_p<number>;

		using TAction = number; // JointDeterministicDecisionRule<TStateDescriptor, TActionDescriptor>;
		using TState = number;
		// using TState = BeliefState<number>;
		// using TState = SerialBeliefState;
		// using TState = BeliefStateGraph_p<number, number>;
		

		// std::shared_ptr<SolvableByHSVI<TState, TAction>> omdp_world = std::make_shared<BeliefMDP<TState, TAction>>(filename);
		auto omdp_world = std::make_shared<DiscreteMDP>(filename);

		std::cout << omdp_world->getDiscount() << std::endl;
		std::cout << omdp_world->getRewardSpace()->getMinReward() << std::endl;
		std::cout << omdp_world->getRewardSpace()->getMaxReward() << std::endl;
		std::cout << omdp_world->getStateSpace() << std::endl;
		std::cout << omdp_world->getActionSpace() << std::endl;

		// Set params in the environment
		// omdp_world->getUnderlyingProblem()->setDiscount(discount);
		// omdp_world->getUnderlyingProblem()->setPlanningHorizon(horizon);

		// // Instanciate initializers
		// auto lb_init = std::make_shared<MinInitializer<TState, TAction>>();
		// auto ub_init = std::make_shared<MaxInitializer<TState, TAction>>();

		// // Instanciate the max-plan representation of the lower bound
		// //auto lower_bound = std::make_shared<MaxPlanValueFunction<TState, TAction>>(omdp_world, horizon, lb_init); //
		// auto lower_bound = std::make_shared<MappedValueFunction<TState, TAction>>(omdp_world, horizon, lb_init);

		// // Instanciate the Tabular version for the upper bound
		// auto upper_bound = std::make_shared<MappedValueFunction<TState, TAction>>(omdp_world, horizon, ub_init);

		// auto algo = std::make_shared<HSVI<TState, TAction>>(omdp_world, lower_bound, upper_bound, horizon, error, trial, "");

		// algo->initialize();
		// algo->solve();
	}
	catch (exception::Exception &e)
	{
		std::cout << "!!! Exception: " << e.what() << std::endl;
	}

} // END main
