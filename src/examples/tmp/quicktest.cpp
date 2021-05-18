#include <iostream>

#include <sdm/exception.hpp>
#include <sdm/algorithms.hpp>
#include <sdm/world/serialized_belief_mdp.hpp>
#include <sdm/core/state/serialized_belief_state.hpp>

#include <sdm/utils/value_function/tabular_value_function.hpp>
#include <sdm/utils/value_function/initializers.hpp>



using namespace sdm;

int main(int argc, char **argv)
{
	std::string filename;
    number horizon;
    number discount = 1;
    double error = 0.00001;
    number trials = 100;

	if (argc > 2)
	{
		filename = argv[1];
		horizon = std::atoi( argv[2] );
	}

	else
	{
		std::cerr << "Error: Require 1 input file." << std::endl;
		return 1;
	}

	try
	{
	}
	catch (exception::Exception &e)
	{
		std::cout << "!!! Exception: " << e.what() << std::endl;
	}

	return 0;
} // END main