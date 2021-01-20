#include <iostream>
#include <cassert>

// #include <sdm/world/decpomdp.hpp>
// #include <sdm/algorithms.hpp>
// #include <sdm/core/state/history.hpp>
// #include <sdm/core/state/history_tree.hpp>
// #include <sdm/core/state/jhistory_tree.hpp>
// #include <sdm/utils/struct/tree.hpp>
// #include <sdm/utils/linear_algebra/sdms_vector.hpp>
// #include <sdm/utils/decision_rules/joint.hpp>
#include <sdm/world/ndpomdp.hpp>
#include <sdm/common.hpp>
#include <sdm/parser/exception.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    char const *filename;

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

    state x = ndpomdp.init();

    std::cout << "Initial State : " << x << std::endl;
    std::uniform_int_distribution<number> random_action_gen(0, ndpomdp.getNumJActions() - 1);

    for (int i = 0; i < 100; i++)
    {
        number random_jaction = random_action_gen(sdm::common::global_urng()); // policy.getAction()
        std::cout << "Action : " << random_jaction << std::endl;

        std::tuple<std::vector<double>, observation, state> r_w_y = ndpomdp.getDynamicsGenerator(x, random_jaction);
        std::vector<double> rews = std::get<0>(r_w_y);
        std::cout << "Reward : ";
        for (auto r : rews)
        {
            std::cout << r << "  ";
        }
        std::cout << "\n";

        number x = std::get<2>(r_w_y);
        number jobservation = std::get<1>(r_w_y);

        std::cout << "Next State : " << x << std::endl
                  << std::endl;
    }

    // std::cout << ndpomdp.getNumStates() << std::endl;
    // std::cout << ndpomdp.getNumAgents() << std::endl;
    // std::cout << ndpomdp.getNumObservations(0) << std::endl;
    // std::cout << ndpomdp.getActionSpace().getNumJElements() << std::endl;
    // std::cout << ndpomdp.getReward(3, 0, 1, 0, 0) << std::endl;
    // std::cout << ndpomdp.getReward(3, 0, 1, 0, 1) << std::endl;
    // std::cout << ndpomdp.getReward(3, 0, 1, 1, 0) << std::endl;
    // std::cout << ndpomdp.getReward(3, 0, 1, 1, 1) << std::endl;

    // std::cout << ndpomdp.getActionSpace().getSpace(0).getNumElements() << std::endl;
    // std::cout << ndpomdp.getNumActions(0) << std::endl;
    // std::cout << ndpomdp.getNumActions(1) << std::endl;
    // std::cout << ndpomdp.getNumActions(2) << std::endl;
    // std::cout << ndpomdp.getNumActions(3) << std::endl;

    // ndpomdp.setupDynamicsGenerator();

    // number s = ndpomdp.init();
    // std::cout << s << std::endl;

    // std::cout << ndpomdp.getReward(<<std::endl;
    // !!!!!!!!!!!  HISTORY TREE   !!!!!!!!!!!

    // JointHistoryTree_p<number> history = std::make_shared<JointHistoryTree<number>>(2, 2);

    // Joint<number> j_obs({1,2});
    // history = history->expand(j_obs);
    // history = history->expand(Joint<number>({4,2}));
    // history = history->expand(Joint<number>({1,1}));
    // history = history->expand(Joint<number>({3,2}));
    // history = history->expand(Joint<number>({4,2}));
    // history = history->expand(Joint<number>({3,3}));
    // history = history->expand(Joint<number>({1,2}));

    // std::cout << *history->getOrigin();
    // std::cout << *history->getIndividualHistory(0);
    // std::cout << *history->getIndividualHistory(1);
    // std::cout << *history->getIndividualHistory(0)->getOrigin();
    // std::cout << *history->getIndividualHistory(1)->getOrigin();

    // !!!!!!!!!!! HSVI !!!!!!!!!!

    // std::shared_ptr<sdm::DecPOMDP> dpomdp_1 = std::make_shared<sdm::DecPOMDP>("../data/world/dpomdp/tiger.dpomdp");
    // std::cout << *dpomdp_1 << std::endl;
    // std::shared_ptr<HSVI<number, number>> hsvi = sdm::algo::makeMappedHSVI<number, number>(dpomdp_1, 0.75, 0.001, 15);
    // hsvi->do_solve();

    return 0;
}
