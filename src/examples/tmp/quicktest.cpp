#include <sdm/world/serialized_occupancy_mdp.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
	// std::string problem_path("../data/world/dpomdp/tiger.dpomdp");
	// number horizon(2);

	// using TState = SerializedState;
	// using TAction = number;

	// auto serial_MDP = std::make_shared<SerializedMMDP<TState, TAction>>(problem_path, horizon);
	// auto underlying_problem = serial_MDP->getUnderlyingProblem();

	// std::cout<<"\n getState : "<<underlying_problem->getStateSpace();
	// std::cout<<"\n getState : "<<underlying_problem->getSerialStateSpace(); // A enlever

	// int number_test_limit= 5;
	// int number_test = 0;
	// do
	// {
	// 	number_test ++;

	// 	auto serial_state = underlying_problem->getStateSpace()->sample();
	// 	number serial_action = underlying_problem->getActionSpace()->sample();

	// 	auto set_next_serial_states = underlying_problem->getReachableSerialStates(serial_state,serial_action);
	// 	std::cout<<"\n next_serial_state : ";
	// 	for(auto const &next_serial_state : set_next_serial_states)
	// 	{
	// 		std::cout<<next_serial_state<<" ";
	// 	}

	// } while (number_test_limit > number_test);

} // END main
