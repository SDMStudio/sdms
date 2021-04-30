#include <iostream>

#include <sdm/world/serialized_mpomdp.hpp>
using namespace sdm;

int main(int argc, char **argv)
{

	// do
	// {
	// 	number_test ++;

<<<<<<< HEAD
		// Instanciate the max-plan representation of the lower bound
		//auto lower_bound_maxplan = std::make_shared<MaxPlanValueFunction<TState, TAction>>(omdp_world, horizon, lb_init); //
		auto lower_bound_tabular = std::make_shared<MappedValueFunction<TState, TAction>>(omdp_world, horizon, lb_init);
 
		// Instanciate the Tabular version for the upper bound
		auto upper_bound = std::make_shared<MappedValueFunction<TState, TAction>>(omdp_world, horizon, ub_init);
=======
	// 	auto serial_state = underlying_problem->getStateSpace()->sample();
	// 	number serial_action = underlying_problem->getActionSpace()->sample();

	// 	auto set_next_serial_states = underlying_problem->getReachableSerialStates(serial_state,serial_action);
	// 	std::cout<<"\n next_serial_state : ";
	// 	for(auto const &next_serial_state : set_next_serial_states)
	// 	{
	// 		std::cout<<next_serial_state<<" ";
	// 	}
>>>>>>> 2b6b460b02da0427bb71b69db21c656e3053b851

	// } while (number_test_limit > number_test);

} // END main
