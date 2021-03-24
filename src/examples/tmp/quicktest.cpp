#include <iostream>
#include <time.h>

#include <sdm/types.hpp>
#include <sdm/tools.hpp>
#include <sdm/utils/struct/pair.hpp>
// #include <sdm/utils/logging/logger.hpp>
// #include <sdm/utils/struct/vector.hpp>
// #include <sdm/utils/struct/pair.hpp>
// #include <sdm/utils/struct/tuple.hpp>
// #include <sdm/world/world_type.hpp>
// #include <sdm/world/discrete_mdp.hpp>
// #include <sdm/world/discrete_pomdp.hpp>
// #include <sdm/world/discrete_decpomdp.hpp>
// #include <sdm/world/solvable_by_hsvi.hpp>
// #include <sdm/world/belief_mdp.hpp>
// #include <sdm/world/serialized_occupancy_mdp.hpp>

// #include <sdm/utils/linear_algebra/mapped_vector.hpp>
// #include <sdm/world/ndpomdp.hpp>
// #include <sdm/world/serialized_occupancy_mdp.hpp>
#include <fmt/format.h>

using namespace sdm;

int main(int argc, char **argv)
{
    std::string filename;
    number horizon, length_history;

    if (argc > 2)
    {
        filename = argv[1];
        horizon = std::atoi(argv[2]);
        length_history = horizon;

        if (argc > 3)
        {
            length_history = std::atoi(argv[3]);
        }
    }
    else
    {
        std::cerr << "Error:  arg[1] must be an input file, arg[2] must be the horizon, arg[3] is optional (the length of history)." << std::endl;
        return 1;
    }
    // using TState = SerializedOccupancyState<number, JointHistoryTree_p<number>>;
    // using TAction = DeterministicDecisionRule<HistoryTree_p<number>, number>;
    // std::shared_ptr<SolvableByHSVI<TState, TAction>> mdp = std::make_shared<SerializedOccupancyMDP<>>(filename);

    // auto upb = mdp->getUnderlyingProblem();

    // std::cout << *upb->getStateSpace() << std::endl;
    // std::cout << *upb->getActionSpace() << std::endl;
    // std::cout << *upb << std::endl;

    sdm::Pair<number, number> p = std::make_pair(2,3);

    std::cout << p ;
    // print(3, 4.5, "hello");

    // std::apply([](auto&&... args) {((std::cout << args << '\n'), ...);}, t);

    // clock_t t_begin, t_end;

    // std::cout << "#> Parsing DecPOMDP file \"" << filename << "\"\n";
    // number n_agents = 2;

    // using TState = SerializedOccupancyState<number, JointHistoryTree_p<number>>;
    // using TAction = DeterministicDecisionRule<HistoryTree_p<number>, number>;

    // auto somdp = std::make_shared<SerializedOccupancyMDP<TState, TAction>>(filename, length_history);

    // auto hsvi = sdm::algo::makeMappedHSVI<TState, TAction>(somdp, 1.0, 0.1, horizon * n_agents);

    // t_begin = clock();

    // hsvi->do_solve();

    // t_end = clock();
    // float temps = (float)(t_end - t_begin) / CLOCKS_PER_SEC;
    // printf("temps = %f\n", temps);

    // hsvi->do_test();

    // NDPOMDP ndpomdp(filename);

    // std::cout << "--------------------------------" << std::endl;

    // std::cout << ndpomdp.getStateSpace() << std::endl;
    // std::cout << ndpomdp.getActionSpace() << std::endl;
    // std::cout << ndpomdp.getObsSpace() << std::endl;

    return 0;
}