#include <iostream>
#include <cassert>
#include <sdm/core/dpomdp.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/parser/exception.hpp>

int main(int argc, char **argv)
{

	char const *filename;

	if (argc > 1)
	{
		filename = argv[1];
		std::cout << "Parsing file \"" << filename << "\"\n";
	}

	else
	{
		std::cerr << "Error: No input file provided." << std::endl;
		return 1;
	}

	try
	{
		sdm::dpomdp dpomdp_world = sdm::parser::parse_file(filename);
		if (dpomdp_world.getNumObservations() * dpomdp_world.getNumActions() < 1000)
		{
			std::cout << dpomdp_world << "\n\n";
		}
		else
		{
			std::cout << "Huge world !!" << std::endl;
			std::cout << "#> Number States : " << dpomdp_world.getNumStates() << std::endl;
			std::cout << "#> Number Actions : " << dpomdp_world.getNumActions() << std::endl;
			std::cout << "#> Number Observations : " << dpomdp_world.getNumObservations() << std::endl;
		}
	}
	catch (sdm::exception::Except &e)
	{
		std::cout << "!!! Exception: " << e.what() << std::endl;
	}
	return 0;
}
