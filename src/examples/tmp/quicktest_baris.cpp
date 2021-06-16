#include <iostream>
#include <memory>

#include <sdm/exception.hpp>
#include <sdm/parser/parser.hpp>

#include <sdm/algorithms/q_learning.hpp>
#include <sdm/utils/value_function/initializer/initializer.hpp>
#include <sdm/utils/value_function/tabular_qvalue_function.hpp>
#include <sdm/utils/rl/exploration.hpp>


// #include <sdm/world/gym_interface.hpp>

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


// #include <sdm/utils/value_function/tabular_value_function.hpp>
// #include <sdm/utils/value_function/backup/tabular_backup.hpp>

// #include <sdm/utils/value_function/initializer/initializer.hpp>
// #include <sdm/utils/value_function/initializer/mdp_initializer.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    // auto dpomdp = sdm::parser::parse_file("../data/world/dpomdp/boxPushingUAI07.dpomdp");
    // auto dpomdp = sdm::parser::parse_file("../data/world/dpomdp/Grid3x3corners.dpomdp");
    auto dpomdp = sdm::parser::parse_file("../data/world/dpomdp/Grid3x3corners.dpomdp");

    auto state_space = dpomdp->getStateSpace();
    auto action_space = dpomdp->getActionSpace();
    auto reward_space = dpomdp->getReward(); 
    auto state_dynamics = dpomdp->getStateDynamics();

    auto start_distribution = dpomdp->getStartDistribution();
    auto observation_space = dpomdp->getObservationSpace(0);
    auto observation_dynamics = dpomdp->getObservationDynamics();

    number horizon = 10;

    double discount = 1;

    double lr = 0.1;

    std::shared_ptr<GymInterface> gym = std::make_shared<MDP>(state_space, action_space, reward_space, state_dynamics, start_distribution, horizon, 1.);
    // std::shared_ptr<GymInterface> gym = std::make_shared<MPOMDP>(state_space, action_space, observation_space, reward_space, state_dynamics, observation_dynamics, start_distribution, horizon, 1.);

    std::shared_ptr<ZeroInitializer> initializer = std::make_shared<sdm::ZeroInitializer>();

    std::shared_ptr<QValueFunction> q_table = std::make_shared<TabularQValueFunction>(horizon, lr, initializer);

    std::shared_ptr<ZeroInitializer> target_initializer = std::make_shared<sdm::ZeroInitializer>();

    std::shared_ptr<QValueFunction> target_q_table = std::make_shared<TabularQValueFunction>(horizon, lr, target_initializer);

    std::shared_ptr<EpsGreedy> exploration = std::make_shared<EpsGreedy>();

    std::shared_ptr<Algorithm> algorithm = std::make_shared<QLearning>(gym, q_table, target_q_table, exploration, horizon, discount, lr, 1, 10000000);

    algorithm->do_initialize();

    algorithm->do_solve();

    return 0;
}
