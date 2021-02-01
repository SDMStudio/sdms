#include <iostream>
#include <cassert>
#include <sdm/core/item.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/space/multi_space.hpp>
#include <sdm/core/space/function_space.hpp>
#include <sdm/core/state/state.hpp>
// #include <sdm/utils/linear_algebra/sdms_vector.hpp>
// #include <sdm/utils/value_function/tabular_value_function.hpp>
#include <sdm/utils/decision_rules/variations.hpp>
#include <sdm/utils/decision_rules/det_decision_rule.hpp>
#include <sdm/common.hpp>
#include <sdm/world/decpomdp.hpp>
#include <sdm/algorithms.hpp>
#include <sdm/exception.hpp>

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

    auto dpomdp = std::make_shared<DecPOMDP>(filename);

    std::cout << *dpomdp << std::endl;

    DiscreteSpace<number> ssp({0, 1, 2});
    DiscreteSpace<number> ssp2({4, 5, 6});
    MultiDiscreteSpace<number> msp({ssp, ssp2});
    std::cout << msp << std::endl;

    // for (auto v : msp.getAll())
    // {
    //     std::cout << v << std::endl;
    // }
    FunctionSpace<DeterministicDecisionRule<number, number>> dfs(ssp, ssp2);

    for (auto f : dfs.getAll())
    {
        std::cout << f << std::endl;
    }

    // BeliefState b1(3, 0), b2(3, 3);
    // b1[2] = 1;
    // b2[0] = 1;
    // DeterministicDecisionRule<BeliefState, DiscreteAction> dr({b1, b2}, {4, 5});
    // std::cout << dr;

    // std::cout << *j_history->getOrigin() << std::endl;
    // DiscreteSpace<TState> state_space = {b1, b2, b3};
    // DiscreteSpace<DiscreteAction> action_space = {b1, b2, b3};

    // FunctionSpace<TAction> f_space(sp, DiscreteSpace<>);
    // for (auto &f : f_space.getAll())
    // {
    //     std::cout << "--------------------\nDetDR<BeliefState, number>\n";
    //     std::cout << f << std::endl;
    // }
    // std::cout << "Number of generated function : "<< f_space.getAll().size() << std::endl;

    // std::cout << space << std::endl;
    // std::cout << space.getSpace(0)->str();
    // std::cout << space << std::endl;

    // MultiSpace space2 = space;
    // std::cout << space2 << std::endl;
    // std::cout << (space2==space) << std::endl;

    //
    // func_space.getAll();

    // for (auto val : func_space.getAll())
    // {
    //     std::cout << val << std::endl;
    // }
    // std::vector<action> actions;
    // std::vector<BeliefState> beliefs;
    // BeliefState b_tmp;
    // b_tmp[0] = 0.5;
    // b_tmp[1] = 0.5;
    // beliefs.push_back(b_tmp);
    // b_tmp[0] = 0.3;
    // b_tmp[1] = 0.7;
    // beliefs.push_back(b_tmp);

    // variations<std::vector<BeliefState>, DeterministicDecisionRule<BeliefState, action>> generator(beliefs, {2, 2});

    // for (auto it = generator.begin(); it != generator.end(); it = generator.next())
    // {
    //     std::cout << it->str() << std::endl;
    // }

    // std::cout << dpomdp->getActionSpace().getJointElementIndex(JointItem({2,})) << std::endl;

    // std::cout << dpomdp->getActionSpace() << std::endl;

    // TabularValueFunction<BeliefState, number> vf(dpomdp, 0, 10);

    // std::cout << vf << std::endl;

    // auto algo = algo::makeMappedHSVI<BeliefState, number>(dpomdp, 0.75, 0.1, 3, 1000);
    // algo->do_solve();

    // NDPOMDP ndpomdp(filename);

    // state x = ndpomdp.init();

    // std::cout << "Initial State : " << x << std::endl;
    // std::uniform_int_distribution<number> random_action_gen(0, ndpomdp.getNumJActions() - 1);

    // for (int i = 0; i < 100; i++)
    // {
    //     number random_jaction = random_action_gen(sdm::common::global_urng()); // policy.getAction()
    //     std::cout << "Action : " << random_jaction << std::endl;

    //     std::tuple<std::vector<double>, observation, state> r_w_y = ndpomdp.getDynamicsGenerator(x, random_jaction);
    //     std::vector<double> rews = std::get<0>(r_w_y);
    //     std::cout << "Reward : ";
    //     for (auto r : rews)
    //     {
    //         std::cout << r << "  ";
    //     }
    //     std::cout << "\n";

    //     number x = std::get<2>(r_w_y);
    //     number jobservation = std::get<1>(r_w_y);

    //     std::cout << "Next State : " << x << std::endl
    //               << std::endl;
    // }

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
