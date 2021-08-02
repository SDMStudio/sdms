#include <iostream>

#include <memory>
#include <sdm/exception.hpp>
#include <sdm/parser/parser.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    std::string ndpomdp_filename;

    if (argc > 1)
    {
        ndpomdp_filename = argv[1];
    }
    else
    {
        std::cerr << "Error: Require 1 input file." << std::endl;
        return 1;
    }

    // Instanciate a standard MPOMDP from a ".ndpomdp" file
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "#> Instanciate a MPOMDP (from a NDPOMDP file)" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::shared_ptr<MPOMDP> mpomdp_from_ndpomdp = sdm::parser::parse_file(ndpomdp_filename);
    mpomdp_from_ndpomdp->setHorizon(2);

    // --------------------------------------------------
    // ----------- DISPLAY SOME ATTRIBUTES --------------
    // --------------------------------------------------
    std::cout << "#> Description of the problem" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "#> Horizon=" << mpomdp_from_ndpomdp->getHorizon() << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "\033[1;31m#> STATE SPACE (standard version)\033[0m" << std::endl;
    std::cout << *mpomdp_from_ndpomdp->getStateSpace(0) << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "\033[1;31m#> ACTION SPACE (standard version)\033[0m" << std::endl;
    std::cout << *mpomdp_from_ndpomdp->getActionSpace(0) << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "\033[1;31m#> OBSERVATION SPACE (standard version)\033[0m" << std::endl;
    std::cout << *mpomdp_from_ndpomdp->getObservationSpace(0) << std::endl;

} // END main