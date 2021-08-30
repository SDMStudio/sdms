#include <iostream>

#include <sdm/config.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/algorithms/alpha_star.hpp>

using namespace sdm;

int main()
{
	std::string filename = sdm::config::PROBLEM_PATH + "dpomdp/mabc.dpomdp";
	number horizon = 10, discount = 1, memory = 1;

	// Parse the problem file 
	auto problem = sdm::parser::parse_file(filename);
	// Set parameters
	problem->setHorizon(horizon);
	problem->setDiscount(discount);

	// Transform the problem in a solvable way 
	std::shared_ptr<SolvableByHSVI> oMDP = std::make_shared<OccupancyMDP>(problem, memory);

	// Instanciate the algorithm
	auto algo = std::make_shared<AlphaStar>(oMDP);

	// Intialize and solve
	algo->do_initialize();
	algo->do_solve();

	return 0;
} 