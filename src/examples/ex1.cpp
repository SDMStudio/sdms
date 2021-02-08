/**
 * @file ex1.cpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief File that give some exemples on how to manipulate formalisms (MDP, POMDP, DecPOMDP, ZSPOSG, etc)
 * @version 1.0
 * @date 01/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <iostream>
#include <cassert>
#include <sdm/worlds.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/exception.hpp>

int main(int argc, char **argv)
{

	std::string filename;
	std::string filename2;

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

		std::cout << "#> DPOMDP_1\n" << dpomdp_1 << "\n\n";
		std::cout << "#> DPOMDP_2\n" << dpomdp_2 << "\n\n";
		std::cout << "#> DPOMDP_3\n" << dpomdp_3 << "\n\n";

		std::cout << "#> ZSPOSG_1\n" << zsposg_1 << "\n\n";
		std::cout << "#> ZSPOSG_2\n" << zsposg_2 << "\n\n";

		return 0;
	}
	catch (sdm::exception::Except &e)
	{
		std::cout << "!!! Exception: " << e.what() << std::endl;
	}
	return 0;
}
