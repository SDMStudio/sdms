#include <iostream>

#include <sdm/parser/parser.hpp>
#include <sdm/exception.hpp>


#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/core/state/serialized_occupancy_state.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>

#include <sdm/world/serialized_mmdp.hpp>

int main(int argc, char **argv)
{
	using TState = sdm::SerializedState; 
	using TAction = sdm::number;

	std::string a("../data/world/dpomdp/mabc.dpomdp");

	auto serialized_mdp = std::make_shared<sdm::SerializedMMDP<TState, TAction>>(a);

	std::vector<sdm::number> action = {0};
	sdm::SerializedState s(0,action);
	std::cout<<"\n getReachableSerialState"<<serialized_mdp->getReachableSerialStates(s,1);

	return 0;
} // END main