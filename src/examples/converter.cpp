#include <iostream>
#include <cassert>
#include <sdm/world/discrete_decpomdp.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/exception.hpp>

int main(int argc, char **argv)
{
	char const *filename_in; // must be of type .dpomdp
	char const *filename_out; // must have extension .xml or .json

	if (argc > 2)
	{
		filename_in = argv[1];
		filename_out = argv[2];
		std::cout << "#> Converting file " << filename_in << " into " << filename_out << "\n";
	}

	else
	{
		std::cerr << "Error: Require two input files." << std::endl;
		return 1;
	}

	try
	{
		auto dpomdp_world = sdm::parser::parse_file(filename_in);
		dpomdp_world->generateFile(filename_out);
	}
	catch (sdm::exception::Exception &e)
	{
		std::cout << "!!! Exception: " << e.what() << std::endl;
	}
	return 0;
}
