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
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "#> Instanciate a standard MPOMDP" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::shared_ptr<MPOMDP> mpomdp = sdm::parser::parse_file(filename);
    mpomdp->setHorizon(2);

    std::cout << "#> Description of the standard MPOMDP" << std::endl;
    std::cout << "#> Horizon=" << mpomdp->getHorizon() << std::endl;

    std::cout << "\033[1;31m#> ACTION SPACE (standard version)\033[0m" << std::endl;
    for (number timestep = 0; timestep < mpomdp->getHorizon(); timestep++)
    {
        std::cout << "\033[1;34m#> T=" << timestep << " :\033[0m\n"
                  << *mpomdp->getActionSpace(timestep) << std::endl;
    }
    std::cout << "----------------------------------------" << std::endl;

    std::cout << "\033[1;31m#> OBSERVATION SPACE (standard version)\033[0m" << std::endl;
    for (number timestep = 0; timestep < mpomdp->getHorizon(); timestep++)
    {
        std::cout << "\033[1;34m#> T=" << timestep << " :\033[0m\n"
                  << *mpomdp->getObservationSpace(timestep) << std::endl;
    }

    // Serialize the problem (transform the game as it was an extensive-form game from the controller point of view)
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "#> Transform the problem into a Serial MPOMDP" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::shared_ptr<MPOMDPInterface> serial_mpomdp = std::make_shared<SerialMPOMDP>(mpomdp);

    std::cout << "#> Description of the serialized MPOMDP" << std::endl;
    std::cout << "#> Horizon=" << serial_mpomdp->getHorizon() << std::endl;

    std::cout << "\033[1;31m#> ACTION SPACE (standard version)\033[0m" << std::endl;
    for (number timestep = 0; timestep < serial_mpomdp->getHorizon(); timestep++)
    {
        std::cout << "\033[1;34m#> T=" << timestep << " :\033[0m\n"
                  << *serial_mpomdp->getActionSpace(timestep) << std::endl;
    }
    std::cout << "----------------------------------------" << std::endl;

    std::cout << "\033[1;31m#> OBSERVATION SPACE (serialized version)\033[0m" << std::endl;
    for (number timestep = 0; timestep < serial_mpomdp->getHorizon(); timestep++)
    {
        std::cout << "\033[1;34m#> T=" << timestep << " :\033[0m\n"
                  << *serial_mpomdp->getObservationSpace(timestep) << std::endl;
    }
    std::cout << "----------------------------------------" << std::endl;

    return 0;
} // END main
