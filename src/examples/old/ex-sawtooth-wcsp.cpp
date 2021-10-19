
#include <cstdlib>
#include <iostream>

#include <sdm/utils/value_function/initializer/initializers.hpp>

#include <sdm/utils/value_function/vfunction/point_set_value_function.hpp>
#include <sdm/utils/value_function/vfunction/pwlc_value_function.hpp>

#include <sdm/utils/value_function/backup/maxplan_backup.hpp>
#include <sdm/utils/value_function/backup/tabular_backup.hpp>

#include <sdm/utils/value_function/action_selection/wcsp/action_sawtooth_wcsp.hpp>
#include <sdm/utils/value_function/action_selection/wcsp/action_maxplan_wcsp.hpp>

#include <sdm/parser/parser.hpp>
#include <sdm/exception.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/algorithms.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
	std::string filename;
    int horizon = 3;
    int discount = 1;
    double error = 0.00001;
    int trials = 3;
	int truncation = 3;

	if (argc > 1)
	{
		filename = argv[1];
	}

	else
	{
		std::cerr << "Error: Require 1 input file." << std::endl;
		return 1;
	}

	try
	{

        auto problem = sdm::parser::parse_file(filename);
        problem->setHorizon(horizon);
        problem->setDiscount(discount);

		std::shared_ptr<SolvableByHSVI> oMDP = std::make_shared<OccupancyMDP>(problem, (truncation > 0) ? truncation : horizon);

        //Create backup class
		auto tabular_backup = std::make_shared<TabularBackup>(oMDP);
		auto maxplan_backup = std::make_shared<MaxPlanBackup>(oMDP);

        //Create select best action class
		auto action_maxplan_wcsp = std::make_shared<ActionSelectionMaxplanWCSP>(oMDP);
		auto action_sawtooth_wcsp =  std::make_shared<ActionSelectionSawtoothWCSP>(oMDP);

        //Create initialisation bound
        auto init_lb = std::make_shared<MinInitializer>(oMDP);
        auto init_ub = std::make_shared<MDPInitializer>(oMDP, "Pomdp Init");

		// Instanciate bounds
		std::shared_ptr<sdm::ValueFunction> lower_bound = std::make_shared<PWLCValueFunction>(horizon,init_lb,maxplan_backup,action_maxplan_wcsp);
		std::shared_ptr<sdm::ValueFunction> upper_bound = std::make_shared<PointSetValueFunction>(horizon,init_ub,tabular_backup, action_sawtooth_wcsp);

        auto algo = std::make_shared<HSVI>(oMDP, lower_bound, upper_bound, problem->getHorizon(), error, trials);

        algo->initialize();
        algo->solve();
	}
	catch (sdm::exception::Exception &e)
	{
		std::cout << "!!! Exception: " << e.what() << std::endl;
	}

	return 0;
} // END main