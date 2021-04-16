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
        using TActionDescriptor = number;
        using TStateDescriptor = HistoryTree_p<number>;

        using TActionPrescriptor = Joint<DeterministicDecisionRule<TStateDescriptor, TActionDescriptor>>;
        using TStatePrescriptor = OccupancyState<number, JointHistoryTree_p<number>>;

        //--------
        //-------- PARAMETERS
        //--------
        // !!! ATTENTION : faire les tests pour de petits horizons (BeliedMDP horizon <= 5 et OccupancyMDP horizon <= 3) pour 
        // BeliefMDP et OccupancyMDP sinon la complexité mémoire explose
        number horizon = 2;
        
        // Pour de petits horizons, on laisse discount = 1.0
        double discount = 1.0, lr = 0.1;
        number max_step = 100000;


        //--------
        //-------- Ex 1) QLEARNING POUR LA RESOLUTION EXACTE DES DecPOMDPs (as OccupancyMDP)
        //--------
        auto env = std::make_shared<OccupancyMDP<TStatePrescriptor, TActionPrescriptor>>(filename, horizon);
        env->getUnderlyingProblem()->setDiscount(discount);
        env->getUnderlyingProblem()->setPlanningHorizon(horizon);
        env->getUnderlyingProblem()->setupDynamicsGenerator();

        auto algo = sdm::algo::makeQLearning<TStatePrescriptor, TActionPrescriptor>(
            env, "", "", horizon, discount, lr, 1, max_step, "test_qlearn_omdp"
        );
        algo->do_initialize();
        algo->do_solve();

        //--------
        //-------- Ex 2) QLEARNING POUR LA RESOLUTION EXACTE DES POMDPs (as BeliefMDP)
        //--------

        // auto env = std::make_shared<BeliefMDP<>>(filename);
        // env->getUnderlyingProblem()->setDiscount(discount);
        // env->getUnderlyingProblem()->setPlanningHorizon(horizon);
        // env->getUnderlyingProblem()->setupDynamicsGenerator();

        // auto algo = sdm::algo::makeQLearning<BeliefState, number>(env, "", "", horizon, discount, lr, 1, max_step, "test_qlearn_bmdp");
        // algo->do_initialize();
        // algo->do_solve();


        //--------
        //-------- Ex 3) QLEARNING POUR LA RESOLUTION EXACTE DES MDPs
        //--------

        // auto env = std::make_shared<DiscreteMDP>(filename);
        // env->setDiscount(discount);
        // env->setPlanningHorizon(horizon);
        // env->setupDynamicsGenerator();

        // auto algo = sdm::algo::makeQLearning<number, number>(env, "qvalue_name", "initializer_name", horizon, discount, lr, 1, max_step, "test_qlearn");
        // algo->do_initialize();
        // algo->do_solve();

    }
    catch (sdm::exception::Exception &e)
    {
        std::cout << "!!! Exception: " << e.what() << std::endl;
    }
    return 0;
}