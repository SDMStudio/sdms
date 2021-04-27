#include <iostream>

#include <sdm/utils/value_function/max_plan_LP.hpp>

int main(int argc, char **argv)
{

<<<<<<< HEAD
	// std::string filename;

	// if (argc > 1)
	// {
	// 	filename = argv[1];
	// }

	// else
	// {
	// 	std::cerr << "Error: Require 1 input file." << std::endl;
	// 	return 1;
	// }

	// try
	// {
	// 	// Construct DecPOMDP using parser
	// 	std::cout << "#> Parsing file \"" << filename << "\"\n";
	// 	auto omdp_world = sdm::parser::parse_file(filename)->toOccupancyMDP();

    //     // We will show how to expand an initial occupancy state and generate next ones 
    //     int depth = 0, limit = 3;
    //     auto ostate = omdp_world->getInitialState();
    //     auto oaction = omdp_world->getActionSpaceAt(ostate)->sample();

    //     do
    //     {
    // 		std::cout << "#> Print depth \"" << depth << "\"\n";
    // 		std::cout << "#> Print occupancy state \n" << ostate << "\n";
    // 		std::cout << "#> Print joint decision rule \n" << oaction << "\n";

    //         depth ++;
    //         ostate = omdp_world->nextState(ostate, oaction);
    //         oaction = omdp_world->getActionSpaceAt(ostate)->sample();
    //     }while(depth < limit);
    // }
    // catch (sdm::exception::Exception &e)
    // {
    //     std::cout << "!!! Exception: " << e.what() << std::endl;
    // }
=======
	std::string filename;

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
		// Construct DecPOMDP using parser
		std::cout << "#> Parsing file \"" << filename << "\"\n";
		auto omdp_world = sdm::parser::parse_file(filename)->toOccupancyMDP();
		std::cout << "#> omdp_world reference \"" << omdp_world << "\"\n";

        // We will show how to expand an initial occupancy state and generate next ones 
        int depth = 0, limit = 2;
        auto ostate = omdp_world->getInitialState();
        auto oaction = omdp_world->getActionSpaceAt(ostate)->sample();
    	
        std::cout << "#> Print depth \"" << depth << "\"\n";
    	std::cout << "#> Print occupancy state \n" << ostate << "\n";
    	std::cout << "#> Print joint decision rule \n" << oaction << "\n";

        do
        {
            depth ++;
            ostate = omdp_world->nextState(ostate, oaction);
            oaction = omdp_world->getActionSpaceAt(ostate)->sample();

    		std::cout << "#> Print depth \"" << depth << "\"\n";
    		std::cout << "#> Print occupancy state \n" << ostate << "\n";
    		std::cout << "#> Print joint decision rule \n" << oaction << "\n";
        } while(depth < limit);
    }
    catch (sdm::exception::Exception &e)
    {
        std::cout << "!!! Exception: " << e.what() << std::endl;
    }
>>>>>>> fafe39e68d96d2fefd0a3a6f3c90b72a097121d6

    return 0;
} // END main