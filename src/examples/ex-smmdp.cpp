#include <sdm/world/serialized_occupancy_mdp.hpp>

using namespace sdm;

int main(int, char **)
{
	std::string problem_path("../data/world/dpomdp/tiger.dpomdp");
	auto serial_MDP = std::make_shared<SerializedMMDP>(problem_path);
	auto underlying_problem = serial_MDP->getUnderlyingProblem();

	std::cout<<"\n getState : "<<underlying_problem->getStateSpace();
	int limit = 5;
	int i = 0;
	do
	{
		i ++;
		auto serial_state = underlying_problem->getStateSpace()->sample();
		auto serial_action = underlying_problem->getActionSpace()->sample();
		auto set_next_serial_states = underlying_problem->getReachableSerialStates(serial_state,serial_action);
		std::cout<< "\n next_serial_state : ";
		for(auto const &next_serial_state : set_next_serial_states)
		{
			std::cout<< next_serial_state <<" ";
		}
	} while (limit > i);

} // END main
