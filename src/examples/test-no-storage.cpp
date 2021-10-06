// #include <vector>
// #include <sdm/core/distribution.hpp>

// using namespace sdm;

// void f1()
// {
// 	DiscreteDistribution<int> dist;
// 	int n = 100000;
// 	for (int i = 0; i < n; i++)
// 		dist.setProbability(i, 1./n);
// }

// int main()
// {
// 	f1();
// 	return 0;
// }

#include <cstdlib>
#include <iostream>

#include <sdm/config.hpp>
#include <sdm/exception.hpp>
#include <sdm/parser/parser.hpp>

#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/algorithms/planning/hsvi.hpp>

#include <sdm/utils/value_function/backup/tabular_backup.hpp>
#include <sdm/utils/value_function/initializer/mdp_initializer.hpp>
#include <sdm/utils/value_function/action_vf/action_tabulaire.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
	std::string filename = (argc > 1) ? argv[1] : config::PROBLEM_PATH + "dpomdp/mabc.dpomdp";
	number horizon = (argc > 2) ? std::stoi(argv[2]) : 4, truncation = (argc > 3) ? std::stoi(argv[3]) : 3;
	double discount = 1., error = 0.001;
	try
	{
		// auto graph = std::make_shared<Graph2<double, int>>();
		// graph->addNode(1.2);
		// graph->addNode(2.3);
		// graph->addNode(5);
		// graph->addNode(21);
		// graph->addSuccessor(5, 10, 1.2);
		// graph->addSuccessor(5, 1, 5);
		// graph->addSuccessor(5, 3, 2.3);
		// graph->addSuccessor(2.3, 8, 5);
		// graph->addSuccessor(2.3, 9, 1.2);

		// std::cout << graph->getNumNodes() << std::endl;
		// std::cout << graph->getSuccessor(5, 10)->getData() << std::endl;
		// std::cout << graph->getSuccessor(2.3, 10) << std::endl;
		// std::cout << graph->getSuccessor(1.2, 10) << std::endl;

		// for (const auto &node : graph->node_space_)
		// {
		// 	std::cout << node.first << " - " << node.second->getData() << " - succ " << node.second->getNumSuccessors() << " - pred " << node.second->getNumPredecessors() << std::endl;
		// }

		// Parse file into MPOMDP
		auto mdp = sdm::parser::parse_file(filename);
		mdp->setHorizon(horizon);
		mdp->setDiscount(discount);

		// Instanciate the problem
		std::shared_ptr<SolvableByHSVI> hsvi_mdp = std::make_shared<OccupancyMDP>(mdp, (truncation > 0) ? truncation : horizon, true, false, false);
		// std::shared_ptr<SolvableByHSVI> hsvi_mdp = std::make_shared<BeliefMDP>(mdp);

		// Instanciate Initializer
		auto init_lb = std::make_shared<MinInitializer>(hsvi_mdp);
		auto init_ub = std::make_shared<MDPInitializer>(hsvi_mdp, "");

		// Instanciate action selection and backup
		auto tabular_backup = std::make_shared<TabularBackup>(hsvi_mdp);
		auto action_tabular = std::make_shared<ActionVFTabulaire>(hsvi_mdp);

		std::shared_ptr<ValueFunction> lb, ub;
		// Instanciate value functions
		lb = std::make_shared<TabularValueFunction2>(mdp->getHorizon(), init_lb, tabular_backup, action_tabular, false);
		ub = std::make_shared<TabularValueFunction2>(mdp->getHorizon(), init_ub, tabular_backup, action_tabular, true);


		// Instanciate HSVI
		auto algo = std::make_shared<HSVI>(hsvi_mdp, lb, ub, mdp->getHorizon(), error, 10000, "", 1, 1);
		algo->initialize();

		// auto state = hsvi_mdp->getInitialState();
		// 	std::cout << "------Initial State-----------" << std::endl;
		// 	std::cout << *state << std::endl;
		// for (int i = 0; i < horizon; i++)
		// {
		// 	std::cout << "------ACTION-----------" << std::endl;
		// 	auto action = ub->getBestAction(state, i);
		// 	std::cout << *action << std::endl;
		// 	std::cout << "------UPDATE LB-----------" << std::endl;
		// 	lb->updateValueAt(state, action, i);
		// 	std::cout << "------UPDATE UB-----------" << std::endl;
		// 	ub->updateValueAt(state, action, i);

		// 	std::cout << "------NEXT STATE-----------" << std::endl;
		// 	state = hsvi_mdp->nextState(state, action, i, algo);
		// 	std::cout << *state << std::endl;
		// }

		// Initialize and solve the problem
		algo->solve();

		// std::cout << "Belief Graph" << std::endl;
		// std::cout << std::static_pointer_cast<OccupancyMDP>(hsvi_mdp)->getUnderlyingBeliefMDP()->getMDPGraph()->getNumNodes() << std::endl;
		// for (const auto &pair_data_node : *std::static_pointer_cast<OccupancyMDP>(hsvi_mdp)->getUnderlyingBeliefMDP()->getMDPGraph()->node_space_)
		// {
		// 	std::cout << "\tsucc(" << *pair_data_node.first << ")=" << std::static_pointer_cast<OccupancyMDP>(hsvi_mdp)->getUnderlyingBeliefMDP()->getMDPGraph()->getNode(pair_data_node.first)->successors.size() << std::endl;
		// 	std::cout << "\tpred(" << *pair_data_node.first << ")=" << std::static_pointer_cast<OccupancyMDP>(hsvi_mdp)->getUnderlyingBeliefMDP()->getMDPGraph()->getNode(pair_data_node.first)->predecessors.size() << std::endl;
		// }

		// std::cout << "State Space" << std::endl;
		// std::cout << std::static_pointer_cast<OccupancyMDP>(hsvi_mdp)->getUnderlyingBeliefMDP()->state_space_.size() << std::endl;

		// std::cout << "Transition" << std::endl;
		// std::cout << std::static_pointer_cast<OccupancyMDP>(hsvi_mdp)->getUnderlyingBeliefMDP()->transition_probability.size() << std::endl;
		// for (const auto &pair_state_act : std::static_pointer_cast<OccupancyMDP>(hsvi_mdp)->getUnderlyingBeliefMDP()->transition_probability)
		// {
		// 	std::cout << "\ttrans(" << *pair_state_act.first << ")=" << pair_state_act.second.size() << std::endl;
		// }

		// std::cout << "Reward Belief Graph" << std::endl;
		// for (const auto &pair_data_node : *std::static_pointer_cast<OccupancyMDP>(hsvi_mdp)->getUnderlyingBeliefMDP()->reward_graph_->node_space_)
		// {
		// 	std::cout << "\tsucc(" << pair_data_node.first << ")=" << std::static_pointer_cast<OccupancyMDP>(hsvi_mdp)->getUnderlyingBeliefMDP()->reward_graph_->getNode(pair_data_node.first)->successors.size() << std::endl;
		// 	std::cout << "\tpred(" << pair_data_node.first << ")=" << std::static_pointer_cast<OccupancyMDP>(hsvi_mdp)->getUnderlyingBeliefMDP()->reward_graph_->getNode(pair_data_node.first)->predecessors.size() << std::endl;
		// }

		// std::cout << "Occupancy Graph" << std::endl;
		// std::cout << std::static_pointer_cast<OccupancyMDP>(hsvi_mdp)->getMDPGraph()->getNumNodes() << std::endl;
		// for (const auto &pair_data_node : *std::static_pointer_cast<OccupancyMDP>(hsvi_mdp)->getMDPGraph()->node_space_)
		// {
		// 	std::cout << "\tsucc(" << *pair_data_node.first << ")=" << std::static_pointer_cast<OccupancyMDP>(hsvi_mdp)->getMDPGraph()->getNode(pair_data_node.first)->successors.size() << std::endl;
		// 	std::cout << "\tpred(" << *pair_data_node.first << ")=" << std::static_pointer_cast<OccupancyMDP>(hsvi_mdp)->getMDPGraph()->getNode(pair_data_node.first)->predecessors.size() << std::endl;
		// }

		// std::cout << "State Space" << std::endl;
		// std::cout << std::static_pointer_cast<OccupancyMDP>(hsvi_mdp)->state_space_.size() << std::endl;

		// std::cout << "Transition" << std::endl;
		// std::cout << std::static_pointer_cast<OccupancyMDP>(hsvi_mdp)->transition_probability.size() << std::endl;
		// for (const auto &pair_state_act : std::static_pointer_cast<OccupancyMDP>(hsvi_mdp)->transition_probability)
		// {
		// 	std::cout << "\ttrans(" << *pair_state_act.first << ")=" << pair_state_act.second.size() << std::endl;
		// }

		// std::cout << "Reward Occupancy Graph" << std::endl;
		// for (const auto &pair_data_node : *std::static_pointer_cast<OccupancyMDP>(hsvi_mdp)->reward_graph_->node_space_)
		// {
		// 	std::cout << "\tsucc(" << pair_data_node.first << ")=" << std::static_pointer_cast<OccupancyMDP>(hsvi_mdp)->reward_graph_->getNode(pair_data_node.first)->successors.size() << std::endl;
		// 	std::cout << "\tpred(" << pair_data_node.first << ")=" << std::static_pointer_cast<OccupancyMDP>(hsvi_mdp)->reward_graph_->getNode(pair_data_node.first)->predecessors.size() << std::endl;
		// }

		// std::cout << "" << std::endl;
		// for (const auto &pair_state_value : lb->getSupport())
		// {
		// }
	}
	catch (exception::Exception &e)
	{
		std::cout << "!!! Exception: " << e.what() << std::endl;
	}

	return 0;
} // END main