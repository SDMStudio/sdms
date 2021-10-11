
#include <cstdlib>
#include <iostream>

#include <sdm/config.hpp>
#include <sdm/exception.hpp>
#include <sdm/parser/parser.hpp>

#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/algorithms/planning/hsvi.hpp>

#include <sdm/utils/value_function/backup/tabular_backup.hpp>
#include <sdm/utils/value_function/action_vf/action_tabulaire.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
	std::string filename = (argc > 1) ? argv[1] : config::PROBLEM_PATH + "dpomdp/mabc.dpomdp";
	number horizon = 10, truncation = 1;
	double error = 0.00001, discount = 1.;
	try
	{
		// Parse file into MPOMDP
		auto mdp = sdm::parser::parse_file(filename);
		mdp->setHorizon(horizon);
		mdp->setDiscount(discount);

		// Instanciate the problem
		std::shared_ptr<SolvableByHSVI> hsvi_mdp = std::make_shared<OccupancyMDP>(mdp, (truncation > 0) ? truncation : horizon, true, true, true);

		// Instanciate Initializer
		auto init_lb = std::make_shared<MinInitializer>(hsvi_mdp);
		auto init_ub = std::make_shared<MaxInitializer>(hsvi_mdp);

		// Instanciate action selection and backup
		auto tabular_backup = std::make_shared<TabularBackup>(hsvi_mdp);
		auto action_tabular = std::make_shared<ActionVFTabulaire>(hsvi_mdp);

		std::shared_ptr<ValueFunction> lb, ub;
		// Instanciate value functions
		lb = std::make_shared<TabularValueFunction>(mdp->getHorizon(), init_lb, tabular_backup, action_tabular, false);
		ub = std::make_shared<TabularValueFunction>(mdp->getHorizon(), init_ub, tabular_backup, action_tabular, true);

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