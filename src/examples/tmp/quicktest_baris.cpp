// #include <iostream>

// #include <memory>
// #include <sdm/exception.hpp>

// #include <sdm/algorithms/hsvi.hpp>

// #include <sdm/core/action/action.hpp>
// #include <sdm/core/action/base_action.hpp>
// #include <sdm/core/state/state.hpp>
// #include <sdm/core/state/base_state.hpp>
// #include <sdm/core/state/serialized_state.hpp>
// #include <sdm/core/base_observation.hpp>

// #include <sdm/core/space/discrete_space.hpp>
// #include <sdm/core/space/multi_discrete_space.hpp>
// #include <sdm/core/distribution.hpp>
// #include <sdm/core/reward/tabular_reward.hpp>
// #include <sdm/core/dynamics/tabular_state_dynamics.hpp>

// #include <sdm/world/mdp.hpp>
// #include <sdm/world/mmdp.hpp>
// #include <sdm/world/solvable_by_mdp.hpp>
// #include <sdm/world/serialized_mmdp.hpp>
// #include <sdm/world/mpomdp.hpp>
// #include <sdm/world/pomdp.hpp>
// #include <sdm/world/belief_mdp.hpp>

// #include <sdm/parser/parser.hpp>

// #include <sdm/utils/value_function/tabular_value_function.hpp>
// #include <sdm/utils/value_function/backup/tabular_backup.hpp>

// #include <sdm/utils/value_function/initializer/initializer.hpp>
// #include <sdm/utils/value_function/initializer/mdp_initializer.hpp>

// using namespace sdm;

int main(int argc, char **argv)
{
    // auto mdp_tiger = sdm::parser::parse_file("../data/world/dpomdp/tiger.dpomdp");
    // // auto mdp_tiger = sdm::parser::parse_file("../data/world/ndpomdp/example4_3-1.ndpomdp");

    // auto state_space = mdp_tiger->getStateSpace(); //std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{state_0, state_1, state_2, state_3});
    // auto action_space = mdp_tiger->getActionSpace(); //= std::make_shared<MultiDiscreteSpace>(std::vector<std::shared_ptr<Space>>{single_action_space, single_action_space});
    // auto rew = mdp_tiger->getReward(); 
    // auto dynamics = mdp_tiger->getStateDynamics();

    // auto start_distrib = mdp_tiger->getStartDistribution();
    // auto obs_space = mdp_tiger->getObservationSpace(0);
    // auto obs_dynamics = mdp_tiger->getObservationDynamics();

    // number horizon = 5;

    // // Creation of the MMDP
    // auto mdp = std::make_shared<MDP>(state_space, action_space, rew, dynamics,start_distrib,horizon,1.);

    // // Creation of HSVI problem and Resolution 
    // std::shared_ptr<SolvableByHSVI> hsvi_mdp = std::make_shared<SolvableByMDP>(mdp);

    // // horizon = horizon * mdp->getNumAgents();
    // auto tabular_backup = std::make_shared<TabularBackup>(hsvi_mdp);

    // auto init_lb = std::make_shared<MinInitializer>(hsvi_mdp);
    // auto init_ub = std::make_shared<MaxInitializer>(hsvi_mdp);
    // // auto init_ub = std::make_shared<MDPInitializer>(hsvi_mdp, "Hsvi",0.01,1000);

    // auto lb = std::make_shared<TabularValueFunction>(horizon,init_lb,tabular_backup);
    // auto ub = std::make_shared<TabularValueFunction>(horizon,init_ub,tabular_backup);

    // auto algo = std::make_shared<HSVI>(hsvi_mdp, lb, ub, horizon, 0.01);
    // algo->do_initialize();
    // std::cout << *algo->getLowerBound() << std::endl;
    // std::cout << *algo->getUpperBound() << std::endl;
    // algo->do_solve();

    return 0;
}
