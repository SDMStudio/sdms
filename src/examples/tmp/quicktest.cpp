#include <sdm/world/serialized_occupancy_mdp.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
	// std::string problem_path("../data/world/dpomdp/tiger.dpomdp");
	// number horizon(2);

	// auto underlying_problem = std::make_shared<SerializedMPOMDP>(problem_path);
	// //auto underlying_problem = serial_POMDP->getUnderlyingProblem();

	// std::cout<<"\n getState : "<<underlying_problem->getStateSpace();

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

	std::vector<std::vector<number>> all_item = {{0,2,3},{1,5}};
	MultiDiscreteSpace<number> space(all_item);

	std::cout<<"\n "<<space.contains(Joint<number>({0,1}));
	std::cout<<"\n "<<space.contains(Joint<number>({0,2}));

} // END main
