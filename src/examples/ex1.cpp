#include <iostream>
#include <cassert>
#include <sdm/common.hpp>
#include <sdm/worlds.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/core/multi_discrete_space.hpp>
#include <sdm/core/space.hpp>
#include <sdm/utils/decision_rules/joint.hpp>
#include <sdm/utils/decision_rules/variations.hpp>
#include <sdm/parser/exception.hpp>

int main(int argc, char **argv)
{

	char const *filename;
	char const *filename2;

	if (argc > 2)
	{
		filename = argv[1];
		filename2 = argv[2];
	}

	else
	{
		std::cerr << "Error: Require 2 input file." << std::endl;
		return 1;
	}

	try
	{
		// Construct DecPOMDP from filename (use parser)
		std::cout << "#> Build DecPOMDP from file \"" << filename << "\"\n";
		sdm::DecPOMDP dpomdp_1(filename);

		// Construct DecPOMDP using parser
		std::cout << "#> Parsing file \"" << filename << "\"\n";
		sdm::DecPOMDP dpomdp_2 = sdm::parser::parse_file(filename);

		// Copie DecPOMDP into another DecPOMDP
		sdm::DecPOMDP dpomdp_3;
		dpomdp_3 = dpomdp_2;

		// Construct ZSPOSG from DecPOMDP problem
		sdm::ZSPOSG zsposg_1(dpomdp_2);

		// Construct ZSPOSG from filename (use parser)
		// sdm::ZSPOSG zsposg_3("../data/world/dpomdp/mabc.zsposg");

		// Construct ZSPOSG using parser
		std::cout << "#> Parsing file \"" << filename2 << "\"\n";
		sdm::ZSPOSG zsposg_2 = sdm::parser::parse_file(filename2);

		std::cout << "#> DPOMDP_1" << dpomdp_1 << "\n\n";
		std::cout << "#> DPOMDP_2" << dpomdp_2 << "\n\n";
		std::cout << "#> DPOMDP_3" << dpomdp_3 << "\n\n";

		std::cout << "#> ZSPOSG_1" << zsposg_1 << "\n\n";
		std::cout << "#> ZSPOSG_2" << zsposg_2 << "\n\n";

		return 0;
	}
	catch (sdm::exception::Except &e)
	{
		std::cout << "!!! Exception: " << e.what() << std::endl;
	}
	return 0;
}
