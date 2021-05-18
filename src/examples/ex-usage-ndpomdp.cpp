#include <iostream>

#include <memory>
#include <sdm/types.hpp>
#include <sdm/exception.hpp>
#include <sdm/world/networked_distributed_pomdp.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
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
        // Construct OccupancyMDP using parser
        auto ndpomdp = std::make_shared<NetworkedDistributedPOMDP>(filename);

        if (ndpomdp->getNumAgents() < 8)
        {
            // std::cout << "----------- XML ------------" << ndpomdp->toXML() << std::endl;
            std::cout << ndpomdp->toStdFormat() << std::endl;
        }

        std::cout << "#> Number of Agents : " << ndpomdp->getNumAgents() << std::endl;
        std::cout << "#> State Space : " << *ndpomdp->getStateSpace() << std::endl;
        std::cout << "#> Action Space : \n" << *ndpomdp->getActionSpace() << std::endl;
        std::cout << "#> Observation Space : \n" << *ndpomdp->getObsSpace() << std::endl;
        
    }
    catch (exception::Exception &e)
    {
        std::cout << "!!! Exception: " << e.what() << std::endl;
    }
} // END main