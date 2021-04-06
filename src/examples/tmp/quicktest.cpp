#include <iostream>
#include <fstream>
#include <time.h>

#include <sdm/types.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/core/states.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/utils/value_function/initializer.hpp>
#include <sdm/utils/value_function/max_plan_vf.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
	char const *filename;
    if (argc > 1)
    {
        filename = argv[1];
        std::cout << "#> Parsing file \"" << filename << "\"\n";
    }

    else
    {
        std::cerr << "Error: No input file provided." << std::endl;
        return 1;
    }

    try
    {

        using TState = BeliefState;
        using TAction = number;

        number horizon = 2;
        auto beliefMDP = std::make_shared<BeliefMDP<TState, TAction>>(filename);
        auto lb_init = std::make_shared<MinInitializer<TState, TAction>>();

        auto lower_bound = std::make_shared<sdm::MaxPlanValueFunction<TState, TAction>>(beliefMDP, horizon, lb_init);

        lower_bound->initialize();
        std::cout << lower_bound->str() << std::endl;

        lower_bound->updateValueAt(beliefMDP->getInitialState(), 0);
        std::cout << lower_bound->size() << std::endl;
        std::cout << lower_bound->str() << std::endl;
    }
    catch (sdm::exception::Exception &e)
    {
        std::cout << "!!! Exception: " << e.what() << std::endl;
    }
    return 0;
}