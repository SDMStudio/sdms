#include <sdm/world/serialized_occupancy_mdp.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
	std::string problem_path("../data/world/dpomdp/tiger.dpomdp");
	auto serial_MPOMDP = std::make_shared<SerializedMPOMDP>(problem_path);

	std::cout<<"\n getState : "<<serial_MPOMDP->getStateSpace()->str();

	int test_limit= 5;
	int test = 0;
	do
	{
		test ++;

		auto serial_state = serial_MPOMDP->getStateSpace()->sample();
		auto serial_state_next = serial_MPOMDP->getStateSpace()->sample();
		auto joint_obs = serial_MPOMDP->getObsSpace()->sample();
		number serial_action = serial_MPOMDP->getActionSpace()->sample();

		auto set_next_serial_states = serial_MPOMDP->getReachableSerialStates(serial_state,serial_action);
		std::cout<<"\n next_serial_state : ";
		for(auto const &next_serial_state : set_next_serial_states)
		{
			std::cout<<next_serial_state<<" ";
		}

		auto set_next_obs = serial_MPOMDP->getReachableObservations(serial_state,serial_action,serial_state_next);
		std::cout<<"\n next_obs : ";
		for(auto const &next_obs : set_next_obs)
		{
			std::cout<<next_obs<<" ";
		}

		std::cout<<"\n serialized_state : "<<serial_state<<", serial_action : "<<serial_action<<", joint_obs : "<<joint_obs<<", next_serialized_state : "<<serial_state_next;
		std::cout<<"\n ObsProbability : "<<serial_MPOMDP->getObservationProbability(serial_state,serial_action,joint_obs,serial_state_next);
		std::cout<<"\n Dynamics : "<<serial_MPOMDP->getDynamics(serial_state,serial_action,joint_obs,serial_state_next);


	} while (test_limit > test);

} // END main
