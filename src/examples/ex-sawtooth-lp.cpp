
#include <cstdlib>
#include <iostream>

#include <sdm/utils/value_function/initializer/initializers.hpp>

#include <sdm/utils/value_function/point_set_value_function.hpp>
#include <sdm/utils/value_function/hyperplan_value_function.hpp>

#include <sdm/utils/value_function/backup/maxplan_backup.hpp>
#include <sdm/utils/value_function/backup/tabular_backup.hpp>

#include <sdm/utils/value_function/action_vf/action_sawtooth_lp.hpp>
#include <sdm/utils/value_function/action_vf/action_maxplan_lp.hpp>

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
    int trials = 50;
	int truncation = 2;

	TypeOfResolution type_of_resolution = TypeOfResolution::BigM;
	TypeSawtoothLinearProgram type_of_linear_program;

	auto ValueBigM = 1000;

	if (argc > 2)
	{
		filename = argv[1];
		if(strcmp( argv[2],"Full") ==0)
		{
			type_of_linear_program = TypeSawtoothLinearProgram::PLAIN_SAWTOOTH_LINER_PROGRAMMING;
		}else if(strcmp( argv[2],"Relaxed")==0)
		{
			type_of_linear_program = TypeSawtoothLinearProgram::RELAXED_SAWTOOTH_LINER_PROGRAMMING;
		}
		else if(strcmp(argv[2],"RelaxedV2") == 0)
		{
			type_of_linear_program = TypeSawtoothLinearProgram::RELAXED_V2_SAWTOOTH_LINER_PROGRAMMING;
		}
	}
	else
	{
		std::cerr << "Error: Require 2 input, the file path and the type of Linear Program (Full, Relaxed, RelaxedV2)." << std::endl;
		return 1;
	}

	try
	{

        auto problem = sdm::parser::parse_file(filename);
        problem->setHorizon(horizon);
        problem->setDiscount(discount);

		std::shared_ptr<SolvableByHSVI> oMDP = std::make_shared<OccupancyMDP>(problem, (truncation > 0) ? truncation : horizon, true, true, true);

		auto tabular_backup = std::make_shared<TabularBackup>(oMDP);
		auto maxplan_backup = std::make_shared<MaxPlanBackup>(oMDP);

		auto action_maxplan_lp = std::make_shared<ActionVFMaxplanLP>(oMDP);
		auto action_sawtooth_lp =  std::make_shared<ActionVFSawtoothLP>(oMDP, type_of_resolution,ValueBigM,type_of_linear_program);

        auto init_lb = std::make_shared<MinInitializer>(oMDP);
        auto init_ub = std::make_shared<MDPInitializer>(oMDP, "Pomdp Init");

		// Instanciate bounds
		std::shared_ptr<sdm::ValueFunction> lower_bound = std::make_shared<HyperplanValueFunction>(horizon,init_lb,maxplan_backup,action_maxplan_lp);
		std::shared_ptr<sdm::ValueFunction> upper_bound = std::make_shared<PointSetValueFunction>(horizon,init_ub,tabular_backup, action_sawtooth_lp);

        auto algo = std::make_shared<HSVI>(oMDP, lower_bound, upper_bound, problem->getHorizon(), error, trials);

        algo->do_initialize();
        algo->do_solve();
	}
	catch (exception::Exception &e)
	{
		std::cout << "!!! Exception: " << e.what() << std::endl;
	}

	return 0;
} // END main