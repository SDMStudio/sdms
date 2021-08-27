
#include <cstdlib>
#include <iostream>

#include <sdm/config.hpp>
#include <sdm/exception.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/algorithms/alpha_star.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
	std::string filename = (argc > 1) ? argv[1] : config::PROBLEM_PATH + "dpomdp/mabc.dpomdp";
	int horizon = 10;
	int discount = 1;
	double error = 0.00001;
	int truncation = 1;
	try
	{
		auto problem = sdm::parser::parse_file(filename);
		problem->setHorizon(horizon);
		problem->setDiscount(discount);

		std::shared_ptr<SolvableByHSVI> oMDP = std::make_shared<OccupancyMDP>(problem, (truncation > 0) ? truncation : horizon, true, true, true);

		auto algo = std::make_shared<AlphaStar>(oMDP);

		algo->do_initialize();
		algo->do_solve();
	}
	catch (exception::Exception &e)
	{
		std::cout << "!!! Exception: " << e.what() << std::endl;
	}

	return 0;
} // END main