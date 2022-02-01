#include <iostream>

#include <sdm/exception.hpp>
#include <sdm/algorithms.hpp>
#include <sdm/world/serial_belief_mdp.hpp>
#include <sdm/core/state/serial_belief_state.hpp>

#include <sdm/utils/value_function/vfunction/tabular_value_function.hpp>
#include <sdm/core/state/base_state.hpp>
#include <sdm/core/state/belief_state.hpp>

using namespace sdm;
using namespace std;

int main(int argc, char **argv)
{
    DiscreteState s1 = make_shared<DiscreteState>(1), s2 = make_shared<DiscreteState>(2), s3 = make_shared<DiscreteState>(3);

    try
    {
        Belief b({s1, s2, s3}, {0.2, 0.4, 0.4});
        std::cout << b << std::endl;


        DecPOMDP problem = parse_file("tiger.dpomdp");
        OccupancyMDP oMDP(problem);

        HSVI hsvi(oMDP, bla,bla,bla);
        hsvi.initialize();
        hsvi.solve();


    }
    catch (exception::Exception &e)
    {
        std::cout << "!!! Exception: " << e.what() << std::endl;
    }

    return 0;
} // END main