#include <iostream>

#include <sdm/config.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/algorithms/planning/alpha_star.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/utils/value_function/update_rule/vupdate/tabular_update.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
	std::string filename = (argc > 1) ? argv[1] : "mabc.dpomdp";
	number horizon = 10, discount = 1, memory = 1;

	// Parse the problem file
	auto problem = sdm::parser::parseMPOMDP(filename);
	// Set parameters
	problem->setHorizon(horizon);
	problem->setDiscount(discount);

	// Transform the problem in a solvable way
	std::shared_ptr<SolvableByHSVI> oMDP = std::make_shared<OccupancyMDP>(problem, memory);

	// Instanciate Initializer
	auto vf_init = std::make_shared<MinInitializer>(oMDP);
	// Instanciate action selection
	auto vf_action_tabular = std::make_shared<ExhaustiveActionSelection>(oMDP);

	// Declare bounds
	std::shared_ptr<ValueFunction> lb, ub;

	// Instanciate lower bound
	auto value_function = std::make_shared<TabularValueFunction>(oMDP, vf_init, vf_action_tabular);

	// Instanciate update operator
	auto vf_update_rule = std::make_shared<TabularUpdate>(value_function);
	value_function->setUpdateRule(vf_update_rule);

	// Instanciate the algorithm
	auto algo = std::make_shared<AlphaStar>(oMDP, value_function);

	// Intialize and solve
	algo->initialize();
	algo->solve();

	return 0;
}