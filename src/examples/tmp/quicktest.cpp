#include <iostream>
#include <cassert>

#include <typeinfo>
#include <sdm/common.hpp>
#include <sdm/world/stochastic_process.hpp>
#include <sdm/core/space/discrete_space.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    std::string filename;

    if (argc > 1)
    {
        filename = argv[1];
        std::cout << "#> Parsing NDPOMDP file \"" << filename << "\"\n";
    }

    else
    {
        std::cerr << "Error: No input file provided." << std::endl;
        return 1;
    }

    DiscreteStochasticProcess<int> process(3);

    process.setStartDistrib({0.0, 1.0, 0.0});
    std::cout << process.init() << std::endl;
    std::cout << process.getStartDistrib()(sdm::common::global_urng()) << std::endl;
    std::cout << process.nextState() << std::endl;


    return 0;
}
