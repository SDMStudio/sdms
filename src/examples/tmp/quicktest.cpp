#include <iostream>
#include <fstream>
#include <time.h>

#include <sdm/types.hpp>
#include <sdm/algorithms.hpp>
#include <sdm/world/discrete_mdp.hpp>
#include <sdm/world/discrete_mmdp.hpp>
#include <sdm/world/discrete_pomdp.hpp>
#include <sdm/world/discrete_decpomdp.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/utils/value_function/initializer.hpp>
#include <sdm/utils/struct/vector.hpp>
#include <sdm/utils/struct/recursive_map.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/linear_algebra/mapped_matrix.hpp>
#include <sdm/utils/value_function/tabular_qvalue_function.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    std::string filename;
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

        // MappedVector<number, MappedVector<number, double>> dmap(0);
        // std::cout << dmap << std::endl;
        // dmap[2][5] = 10.5;
        // std::cout << dmap << std::endl;

        // MappedTensor<number, number, double> dmap = {{2, {{5, 10.2}}};
        MappedQValueFunction<number, number> tab_qval(0, 0.1, std::make_shared<ValueInitializer<number, number>>(3));
        std::cout << tab_qval << std::endl;
        tab_qval.initialize();
        std::cout << tab_qval << std::endl;
        std::cout << tab_qval.getQValueAt(3, 1, 1) << std::endl;
        std::cout << tab_qval.getQValueAt(2, 1, 0) << std::endl;
        std::cout << tab_qval << std::endl;
        tab_qval.updateQValueAt(3, 1, 1, 16);
        tab_qval.updateQValueAt(0, 2, 0, 4);
        std::cout << tab_qval << std::endl;
        std::cout << tab_qval.getQValueAt(3, 1, 1) << std::endl;
        std::cout << tab_qval.getQValueAt(2, 1, 0) << std::endl;
        std::cout << tab_qval << std::endl;

        number horizon = 3;
        double discount = 1.0, lr = 0.1;
        number n_episode = 10, max_step = 10000;
        // auto env = std::make_shared<BeliefMDP<>>(filename);
        auto env = std::make_shared<BeliefMDP<>>(filename);
        // auto env = std::make_shared<DiscretePOMDP>(filename);
        // auto env = std::make_shared<DiscreteDecPOMDP>(filename);

        env->getUnderlyingProblem()->setDiscount(discount);
        env->getUnderlyingProblem()->setPlanningHorizon(horizon);
        auto algo = sdm::algo::makeQLearning<BeliefState, number>(env, "qvalue_name", "initializer_name", horizon, discount, lr, 1, max_step, "test_qlearn");
        algo->do_initialize();
        algo->do_solve();

        // env->setPlanningHorizon(horizon);

        // for (number episode = 0; episode < n_episode; episode++)
        // {
        //     env->reset();
        //     std::cout << "---------------" << std::endl;
        //     for (number step = 0; step < 10; step++)
        //     {
        //         auto action = env->getActionSpaceAt(0)->sample();
        //         auto [next_obs, rewards, done] = env->step(action);
        //         std::cout << "#Ep " << episode +1 << "/" << n_episode << " | Step " << step << " | act=" << action <<" obs=" << next_obs << " rewards=" << rewards << " done=" << done << std::endl;
        //         if (done)
        //         {
        //             break;
        //         }
        //     }
        // }
    }
    catch (sdm::exception::Exception &e)
    {
        std::cout << "!!! Exception: " << e.what() << std::endl;
    }
    return 0;
}