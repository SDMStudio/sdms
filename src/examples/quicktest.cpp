#include <iostream>
#include <cassert>

#include <typeinfo>
// #include <sdm/core/item.hpp>
// #include <sdm/core/space/discrete_space.hpp>
// #include <sdm/core/space/multi_discrete_space.hpp>
// #include <sdm/core/space/multi_space.hpp>
// #include <sdm/core/space/function_space.hpp>
// #include <sdm/core/state/state.hpp>
#include <sdm/algorithms.hpp>
#include <sdm/spaces.hpp>
#include <sdm/core/state/history.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/utils/value_function/max_plan_vf.hpp>
#include <sdm/utils/decision_rules/det_decision_rule.hpp>
#include <sdm/world/decpomdp.hpp>
#include <sdm/world/pomdp.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/exception.hpp>

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

    std::shared_ptr<DecPOMDP> dpomdp = std::make_shared<DecPOMDP>(filename);
    std::cout << *dpomdp << std::endl;

    std::shared_ptr<DecPOMDP> pomdp = dpomdp->toPOMDP();
    std::cout << pomdp << std::endl;

    std::shared_ptr<SolvableByHSVI<BeliefState, number>> bmdp = std::make_shared<BeliefMDP<BeliefState, number, number>>(pomdp);
    auto algo = algo::makeMappedHSVI<BeliefState, number>(bmdp, 0.75, 0.01, 3);
    algo->do_solve();
    algo->do_test();

    // Defines the different types
    // using TObservation = number;
    // using TState = number;
    // using TActionDescriptor = number;

    // using TStateDescriptor = HistoryTree_p<TObservation>;

    // using TActionPrescriptor = Joint<DeterministicDecisionRule<TStateDescriptor, TActionDescriptor>>;
    // using TStatePrescriptor = OccupancyState<TState, JointHistoryTree_p<TObservation>>;

    // number h = 2;
    // double discount = 1;

    // auto algo = sdm::algo::make("mapped_hsvi", filename, discount, 0.01, h, 10000);

    // algo->do_solve();
    // algo->do_test();

    // std::shared_ptr<SolvableByHSVI<TStatePrescriptor, TActionPrescriptor>> oMDP = std::make_shared<OccupancyMDP<TStatePrescriptor, TActionPrescriptor>>(dpomdp, h);
    // // std::shared_ptr<SolvableByHSVI<BeliefState, number>> oMDP = std::make_shared<BeliefMDP<BeliefState, number, number>>(dpomdp);
    // dpomdp->setDiscount(discount);

    // auto lb_init = std::make_shared<sdm::MinInitializer<TStatePrescriptor, TActionPrescriptor>>(dpomdp->getReward().getMinReward(), discount);
    // auto ub_init = std::make_shared<sdm::MaxInitializer<TStatePrescriptor, TActionPrescriptor>>(dpomdp->getReward().getMaxReward(), discount);

    // // std::shared_ptr<sdm::ValueFunction<BeliefState, number>> upper_bound(new sdm::MappedValueFunction<BeliefState, number>(oMDP, h, ub_init));
    // // std::shared_ptr<sdm::ValueFunction<BeliefState, number>> lower_bound(new sdm::MappedValueFunction<BeliefState, number>(oMDP, h, lb_init));
    // std::shared_ptr<sdm::ValueFunction<TStatePrescriptor, TActionPrescriptor>> upper_bound(new sdm::MaxPlanValueFunction<TStatePrescriptor, TActionPrescriptor>(oMDP, h, ub_init));
    // std::shared_ptr<sdm::ValueFunction<TStatePrescriptor, TActionPrescriptor>> lower_bound(new sdm::MaxPlanValueFunction<TStatePrescriptor, TActionPrescriptor>(oMDP, h, lb_init));
    // // std::cout << *upper_bound << std::endl;
    // // std::cout << *lower_bound << std::endl;

    // HSVI<TStatePrescriptor, TActionPrescriptor> algo(oMDP, lower_bound, upper_bound, h, 0.01);
    // algo.do_solve();
    // algo.do_test();

    // oMDP->getReward();

    // using TState = BeliefState;
    // using TAction = number;

    // auto algo = algo::makeMappedHSVI<TState, TAction>(dpomdp, 0.99, 0.001, 6, 1000);
    // auto v_star = algo->do_solve();

    // std::cout << v_star << std::endl;

    // TState b = dpomdp->getStartDistrib();
    // for (int i = 0; i < 6; i++)
    // {
    //     TAction action = v_star->getQValueAt(b, i)->argmax();
    //     b = dpomdp->nextState(b, action);
    // }

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
