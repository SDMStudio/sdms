
#include <cstdlib>
#include <iostream>

#include <sdm/parser/parser.hpp>
#include <sdm/exception.hpp>
#include <sdm/algorithms/backward_induction.hpp>
#include <sdm/world/occupancy_mdp.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
	std::string filename;
    int horizon = 3;
    int discount = 1;
    double error = 0.00001;
	int truncation = 2;

	if (argc > 1)
	{
		filename = argv[1];
	}
	else
	{
		std::cerr << "Error: Require 1 input, the file path." << std::endl;
		return 1;
	}

	try
	{
        auto problem = sdm::parser::parse_file(filename);
        problem->setHorizon(horizon);
        problem->setDiscount(discount);

		std::shared_ptr<SolvableByHSVI> oMDP = std::make_shared<OccupancyMDP>(problem, (truncation > 0) ? truncation : horizon, true, true, true);

        auto algo = std::make_shared<BackwardInduction>(oMDP);

        algo->do_initialize();
        algo->do_solve();
	}
	catch (exception::Exception &e)
	{
		std::cout << "!!! Exception: " << e.what() << std::endl;
	}

	return 0;
} // END main