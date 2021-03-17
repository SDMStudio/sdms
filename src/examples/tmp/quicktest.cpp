#include <iostream>
#include <time.h>

#include <sdm/common.hpp>
#include <sdm/algorithms.hpp>
#include <sdm/parser/parser.hpp>

#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/world/ndpomdp.hpp>
#include <sdm/world/serialized_occupancy_mdp.hpp>

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


    // MappedVector<Joint<number>, number> umv(4);

    // umv[Joint<number>({2,3,7})] = 6;
    // umv[Joint<number>({1,3,7})] = 6;
    // umv[Joint<number>({2,5,9})] = 12;

    // std::cout << umv << std::endl;


    // MappedVector<Joint<number>, number> umv2(3);

    // umv2[Joint<number>({2,3,7})] = 6;
    // umv2[Joint<number>({1,3,7})] = 61;
    // umv2[Joint<number>({2,5,9})] = 2;

    // std::cout << umv2 << std::endl;

    clock_t t_begin, t_end;

    std::cout << "#> Parsing DecPOMDP file \"" << filename << "\"\n";
    number n_agents = 2;

    using TState = SerializedOccupancyState<number, JointHistoryTree_p<number>>;
    using TAction = DeterministicDecisionRule<HistoryTree_p<number>, number>;

    auto somdp = std::make_shared<SerializedOccupancyMDP<TState, TAction>>(filename, length_history);

    auto hsvi = sdm::algo::makeMappedHSVI<TState, TAction>(somdp, 0.9, 0.1, horizon * n_agents);

    t_begin = clock();

    hsvi->do_solve();

    t_end = clock();
    float temps = (float)(t_end - t_begin) / CLOCKS_PER_SEC;
    printf("temps = %f\n", temps);

    hsvi->do_test();

    // NDPOMDP ndpomdp(filename);

    // std::cout << "--------------------------------" << std::endl;

    // std::cout << ndpomdp.getStateSpace() << std::endl;
    // std::cout << ndpomdp.getActionSpace() << std::endl;
    // std::cout << ndpomdp.getObsSpace() << std::endl;

    return 0;
}