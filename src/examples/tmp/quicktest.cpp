#include <iostream>

#include <sdm/parser/parser.hpp>
#include <sdm/exception.hpp>

#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/world/discrete_decpomdp.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>

#include <sdm/world/serialized_mmdp.hpp>
using namespace sdm;

int main(int argc, char **argv)
{

	using TState = SerializedState;
	using TAction = number;

	std::string a("../data/world/dpomdp/tiger.dpomdp");

	auto serialized_mdp = std::make_shared<sdm::SerializedMMDP<TState, TAction>>(a);

	std::cout<<"\n getState "<<serialized_mdp->getUnderlyingProblem()->getStateSpace()->getAll();
	std::cout<<"\n getAction "<<serialized_mdp->getUnderlyingProblem()->getActionSpace()->getAll();

	// auto serialized_pomdp = std::make_shared<sdm::SerializedMPOMDP<TState, TAction>>(a);


	// std::vector<sdm::number> action = {};
	// sdm::SerializedState s(0,action);
	
	// std::vector<number> all_action = {0,0} ;
	// for(auto s : serialized_pomdp->getReachableObservations(Joint<number>(all_action),s))
	// {
	// 	std::cout<<"\n s  :"<<s;
	// }

	//auto serialized_pomdp = std::make_shared<sdm::SerializedMPOMDP<TState, TAction>>(a);

	return 0;
} // END main