#include <iostream>
#include <cassert>

#include <tuple>
#include <typeinfo>
#include <sdm/common.hpp>

#include <sdm/parser/parser.hpp>

#include <sdm/world/ndpomdp.hpp>



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

    NDPOMDP ndpomdp(filename);

    std::cout << "--------------------------------" << std::endl;
    
    std::cout << ndpomdp.getStateSpace() << std::endl;
    std::cout << ndpomdp.getActionSpace() << std::endl;
    std::cout << ndpomdp.getObsSpace() << std::endl;



    return 0;
}