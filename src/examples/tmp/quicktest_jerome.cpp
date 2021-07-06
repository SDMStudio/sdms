#include <iostream>

#include <memory>
#include <sdm/exception.hpp>

// #include <sdm/algorithms/hsvi.hpp>

#include <sdm/core/action/action.hpp>
#include <sdm/core/action/base_action.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/state/base_state.hpp>
#include <sdm/core/state/serialized_state.hpp>
#include <sdm/core/base_observation.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/distribution.hpp>
#include <sdm/core/reward/tabular_reward.hpp>
#include <sdm/core/dynamics/tabular_state_dynamics.hpp>

#include <sdm/world/mdp.hpp>
#include <sdm/world/mmdp.hpp>
#include <sdm/world/solvable_by_mdp.hpp>
#include <sdm/world/serialized_mmdp.hpp>
#include <sdm/world/mpomdp.hpp>
#include <sdm/world/pomdp.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/occupancy_mdp.hpp>

#include <sdm/parser/parser.hpp>

#include <sdm/utils/value_function/tabular_value_function.hpp>
#include <sdm/utils/value_function/point_set_value_function.hpp>
#include <sdm/utils/value_function/hyperplan_value_function.hpp>

#include <sdm/utils/value_function/backup/maxplan_backup.hpp>
#include <sdm/utils/value_function/backup/tabular_backup.hpp>

#include <sdm/utils/value_function/action_vf/action_tabulaire.hpp>
#include <sdm/utils/value_function/action_vf/action_maxplan.hpp>
#include <sdm/utils/value_function/action_vf/action_maxplan_lp.hpp>

#include <sdm/utils/value_function/initializer/initializer.hpp>
#include <sdm/utils/value_function/initializer/mdp_initializer.hpp>
#include <sdm/utils/value_function/initializer/pomdp_initializer.hpp>

#include <sdm/world/serialized_mpomdp.hpp>

#include <sdm/core/state/history_tree.hpp>
// #include <sdm/core/state/jhistory_tree.hpp>

#include <sdm/utils/value_function/initializer/initializers.hpp>

using namespace sdm;


int main(int argc, char **argv)
{
    auto mdp_tiger = sdm::parser::parse_file("../data/world/dpomdp/tiger.dpomdp");

    auto state_space = std::static_pointer_cast<DiscreteSpace>(mdp_tiger->getStateSpace()); //std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{state_0, state_1, state_2, state_3});
    auto action_space = mdp_tiger->getActionSpace(); //= std::make_shared<MultiDiscreteSpace>(std::vector<std::shared_ptr<Space>>{single_action_space, single_action_space});
    auto rew = mdp_tiger->getReward(); 
    auto dynamics = mdp_tiger->getStateDynamics();

    auto start_distrib = mdp_tiger->getStartDistribution();
    auto obs_space = mdp_tiger->getObservationSpace(0);
    auto obs_dynamics = mdp_tiger->getObservationDynamics();

    number horizon = 3;

    // Creation of the MMDP
    auto mpomdp = std::make_shared<MPOMDP>(state_space, action_space,obs_space, rew, dynamics,obs_dynamics,start_distrib,horizon,1.);

    // Creation of HSVI problem and Resolution 
    std::shared_ptr<SolvableByHSVI> hsvi = std::make_shared<OccupancyMDP>(mpomdp,2);

    // horizon = horizon * serial_mmdp->getNumAgents();
    auto tabular_backup = std::make_shared<TabularBackup>(hsvi);
    auto maxplan_backup = std::make_shared<MaxPlanBackup>(hsvi);

    auto action_tabular = std::make_shared<ActionVFTabulaire>(hsvi);
    auto action_maxplan = std::make_shared<ActionVFMaxplan>(hsvi);
    auto action_maxplan_lp = std::make_shared<ActionVFMaxplanLP>(hsvi);

    // auto init_lb = std::make_shared<MinInitializer>(hsvi);
    // auto init_ub = std::make_shared<POMDPInitializer>(hsvi,"");

    auto init_lb = sdm::makeInitializer("Min",hsvi);
    auto init_ub = sdm::makeInitializer("PomdpHsvi",hsvi);

    auto ub = std::make_shared<PointSetValueFunction>(horizon,init_ub,tabular_backup,action_tabular);
    auto lb = std::make_shared<HyperplanValueFunction>(horizon,init_lb,maxplan_backup,action_maxplan_lp); 

    auto algorithm = std::make_shared<HSVI>(hsvi, lb, ub, horizon, 0.01);
    algorithm->do_initialize();

    algorithm->do_solve(); 

    return 0;
} // END main
