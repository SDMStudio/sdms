#include <iostream>

#include <memory>
#include <sdm/exception.hpp>

// #include <sdm/algorithms/hsvi.hpp>

#include <sdm/core/action/action.hpp>
#include <sdm/core/action/base_action.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/state/base_state.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/distribution.hpp>
#include <sdm/core/reward/tabular_reward.hpp>
#include <sdm/core/dynamics/tabular_state_dynamics.hpp>

#include <sdm/world/mdp.hpp>
#include <sdm/world/mmdp.hpp>
#include <sdm/world/solvable_by_mdp.hpp>
#include <sdm/parser/parser.hpp>

#include <sdm/utils/value_function/tabular_value_function.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    auto mdp_tiger = sdm::parser::parse_file("../data/world/dpomdp/tiger.dpomdp");

    // auto state_0 = std::make_shared<DiscreteState>(0);
    // auto state_1 = std::make_shared<DiscreteState>(1);
    // auto state_2 = std::make_shared<DiscreteState>(2);
    // auto state_3 = std::make_shared<DiscreteState>(3);

    auto action_0 = std::make_shared<DiscreteAction>(0);
    auto action_1 = std::make_shared<DiscreteAction>(1);
    auto action_2 = std::make_shared<DiscreteAction>(2);
    auto action_3 = std::make_shared<DiscreteAction>(3);

    std::shared_ptr<Joint<std::shared_ptr<Action>>> jaction = std::make_shared<Joint<std::shared_ptr<Action>>>(std::vector<std::shared_ptr<Action>>{action_0, action_1, action_2, action_1});

    std::cout << "JACTION " << jaction << std::endl;
    std::cout << *jaction << std::endl;

    std::shared_ptr<Item> item = jaction;
    std::cout << "ITEM " << item << std::endl;
    std::cout << *item << std::endl;

    // // auto test = MultiDiscreteSpace<number>();
    // std::cout << *joint_action << std::endl;
    // std::cout << jaction << std::endl;
    // std::shared_ptr<Space> space1 = std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{action_0, action_1, action_2, action_1});
    // std::shared_ptr<Space> space2 = std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{action_0, action_1, action_1, action_1});

    // std::shared_ptr<Space> mdspace = std::make_shared<MultiDiscreteSpace>(std::vector<decltype(space2)>{space1, space2});

    // std::cout << *mdspace << std::endl;

    // for (const auto &values:  *mdspace)
    // {
    //     std::cout << values << std::endl;
    // }

    // for (const Space::iterator_type &values = mdspace->begin(); values != mdspace->end(); values->operator++())
    // {
    //     std::cout << *values << std::endl;
    // }

    auto state_space = mdp_tiger->getStateSpace(); //std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{state_0, state_1, state_2, state_3});
    std::cout << *state_space << std::endl;

    // std::shared_ptr<Space> single_action_space = std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{action_0, action_1, action_2, action_3});
    std::shared_ptr<Space> action_space = mdp_tiger->getActionSpace(); //= std::make_shared<MultiDiscreteSpace>(std::vector<std::shared_ptr<Space>>{single_action_space, single_action_space});

    auto start_distrib = std::make_shared<DiscreteDistribution<std::shared_ptr<State>>>();
    for (const auto &state : *state_space)
    {
        start_distrib->setProbability(std::static_pointer_cast<State>(state), 1. / std::static_pointer_cast<DiscreteSpace>(state_space)->getNumItems());
    }

    auto rew = mdp_tiger->getReward(); 
    for (const auto &state : *state_space) // = state_space->begin(); state != state_space->end(); state = state_space->next())
    {
        for (const auto &action : *action_space) //= action_space->begin(); action != action_space->end(); action = action_space->next())
        {
            std::cout << "Reward(" << *state << ", " << *action << ") = " << rew->getReward(std::static_pointer_cast<State>(state), std::static_pointer_cast<Action>(action), 0) << std::endl;
        }
    }

    auto dynamics = mdp_tiger->getStateDynamics();

    for (const auto &state : *state_space)
    {
        for (const auto &action : *action_space)
        {
            for (const auto &next_state : *state_space)
            {
                std::cout << "T(" << *state << ", " << *action << ", " << *next_state << ") = " << dynamics->getTransitionProbability(std::static_pointer_cast<State>(state), std::static_pointer_cast<Action>(action), std::static_pointer_cast<State>(next_state), 0) << std::endl;
            }
        }
    }

    number horizon = 3;

    auto mdp = std::make_shared<MMDP>(state_space, action_space, rew, dynamics, start_distrib);

    std::cout << "#> NumAgents = " << mdp->getNumAgents() << std::endl;
    std::cout << "#> Discount = " << mdp->getDiscount() << std::endl;
    std::cout << "#> StateSpace = " << *mdp->getStateSpace() << std::endl;
    std::cout << "#> ActionSpace = " << *mdp->getActionSpace() << std::endl;

    std::shared_ptr<SolvableByHSVI> hsvi_mdp = std::make_shared<SolvableByMDP>(mdp);

    auto lb = std::make_shared<MappedValueFunction>(hsvi_mdp, horizon, -1000);
    auto ub = std::make_shared<MappedValueFunction>(hsvi_mdp, horizon, 1000);

    auto algo = std::make_shared<HSVI>(hsvi_mdp, lb, ub, horizon, 0.01);
    algo->do_initialize();
    std::cout << *algo->getLowerBound() << std::endl;
    std::cout << *algo->getUpperBound() << std::endl;
    algo->do_solve();
    std::cout << *algo->getLowerBound() << std::endl;
    std::cout << *algo->getUpperBound() << std::endl;

} // END main
