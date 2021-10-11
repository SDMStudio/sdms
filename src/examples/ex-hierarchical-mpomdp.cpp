#include <iostream>

#include <sdm/types.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/world/hierarchical_mpomdp.hpp>
#include <sdm/world/serial_mpomdp.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    std::string filename = (argc > 1) ? argv[1] : config::PROBLEM_PATH + "dpomdp/tiger.dpomdp";
 

    // Instanciate a standard Multi-agent POMDP
    std::cout << "#> Instanciate a standard MPOMDP" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::shared_ptr<MPOMDPInterface> mpomdp = sdm::parser::parse_file(filename);

    std::cout << "\033[1;31m#> OBSERVATION SPACE (standard MPOMDP version)\033[0m" << std::endl;
    for (number agent_id = 0; agent_id < mpomdp->getNumAgents(); agent_id++)
    {
        std::cout << "\033[1;34m#> Observations Agent " << agent_id << " :\033[0m\n"
                  << *mpomdp->getObservationSpace(agent_id, 0) << std::endl;
    }

    // Hierarchize the problem (put a hierarchy between agents)
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "#> Transform the problem into a Hierarchical MPOMDP" << std::endl;
    std::shared_ptr<MPOMDPInterface> hierarchical_mpomdp = std::make_shared<HierarchicalMPOMDP>(mpomdp);

    std::cout << "----------------------------------------" << std::endl;
    std::cout << "\033[1;31m#> OBSERVATION SPACE (Hierarchical MPOMDP version)\033[0m" << std::endl;
    for (number agent_id = 0; agent_id < mpomdp->getNumAgents(); agent_id++)
    {
        std::cout << "\033[1;34m#> Observations Agent " << agent_id << " :\033[0m\n"
                  << *hierarchical_mpomdp->getObservationSpace(agent_id, 0) << std::endl;
    }

    return 0;
} // END main
