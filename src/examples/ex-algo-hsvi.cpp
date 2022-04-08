
#include <cstdlib>
#include <iostream>

#include <sdm/config.hpp>
#include <sdm/exception.hpp>
#include <sdm/parser/parser.hpp>

#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/algorithms/planning/hsvi.hpp>
#include <sdm/utils/value_function/action_selection/exhaustive_action_selection.hpp>
#include <sdm/utils/value_function/update_rule/vupdate/tabular_update.hpp>
#include <sdm/utils/value_function/vfunction/tabular_value_function.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
	std::string filename = (argc > 1) ? argv[1] : "mabc.dpomdp";
	number horizon = 10, truncation = 1;
	double error = 0.00001, discount = 1.;
	try
	{
		// Parse file into MPOMDP
		auto mdp = sdm::parser::parseMPOMDP(filename);
		mdp->setHorizon(horizon);
		mdp->setDiscount(discount);

		// Instanciate the problem
		std::shared_ptr<SolvableByHSVI> hsvi_mdp = std::make_shared<OccupancyMDP>(mdp, (truncation > 0) ? truncation : horizon, true, true);

		// Instanciate Initializer
		auto lb_init = std::make_shared<MinInitializer>(hsvi_mdp);
		auto ub_init = std::make_shared<MaxInitializer>(hsvi_mdp);

		// Instanciate action selection 
		auto action_tabular = std::make_shared<ExhaustiveActionSelection>(hsvi_mdp);

		// Declare bounds
		std::shared_ptr<ValueFunction> lb, ub;

		// Instanciate lower bound
		lb = std::make_shared<TabularValueFunction>(hsvi_mdp, lb_init, action_tabular);

		// Instanciate lower bound update operator
		auto lb_update_rule = std::make_shared<TabularUpdate>(lb);
		lb->setUpdateRule(lb_update_rule);

		// Instanciate upper bound
		ub = std::make_shared<TabularValueFunction>(hsvi_mdp, ub_init, action_tabular);

		// Instanciate upper bound update operator
		auto ub_update_rule = std::make_shared<TabularUpdate>(ub);
		ub->setUpdateRule(ub_update_rule);

		// Instanciate HSVI
		auto algo = std::make_shared<HSVI>(hsvi_mdp, lb, ub, error, 10000, "", 1, 1);

		// Initialize and solve the problem
		algo->initialize();
		algo->solve();
	}
	catch (exception::Exception &e)
	{
		std::cout << "!!! Exception: " << e.what() << std::endl;
	}

	return 0;
} // END main