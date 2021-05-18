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

		using TState = SerializedBeliefState;
		using TAction = number;
		using TObservation = Joint<number>;

		std::cout << "#> Parsing file \"" << filename << "\"\n";

		std::shared_ptr<SolvableByHSVI<TState, TAction>> serial_belief_MDP = std::make_shared<SerializedBeliefMDP<TState, TAction, TObservation>>(filename);        

		serial_belief_MDP->getUnderlyingProblem()->setDiscount(discount);
		serial_belief_MDP->getUnderlyingProblem()->setPlanningHorizon(horizon);

		if (serial_belief_MDP->isSerialized())
		{
			horizon = horizon * serial_belief_MDP->getUnderlyingProblem()->getNumAgents();
		}

		// Instanciate initializers 
		auto lb_init = std::make_shared<MinInitializer<TState, TAction>>();
		auto ub_init = std::make_shared<MaxInitializer<TState, TAction>>();

		// Instanciate the Tabular version for the lower bound
		std::shared_ptr<sdm::ValueFunction<TState, TAction>> lower_bound = std::make_shared<MappedValueFunction<TState, TAction>>(serial_belief_MDP, horizon, lb_init); 

		// Instanciate the Tabular version for the upper bound 
		std::shared_ptr<sdm::ValueFunction<TState, TAction>> upper_bound = std::make_shared<MappedValueFunction<TState, TAction>>(serial_belief_MDP, horizon, ub_init);

		auto p_algo = std::make_shared<HSVI<TState, TAction>>(serial_belief_MDP, lower_bound, upper_bound, horizon, error, trials, "Example-Serial-Belief-MDP");

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