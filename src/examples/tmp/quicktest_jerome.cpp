#include <iostream>

#include <memory>
#include <sdm/exception.hpp>

// #include <sdm/algorithms/hsvi.hpp>

#include <sdm/core/action/action.hpp>
#include <sdm/core/action/base_action.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/state/base_state.hpp>
#include <sdm/core/state/serialized_state.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/distribution.hpp>
#include <sdm/core/reward/tabular_reward.hpp>
#include <sdm/core/dynamics/tabular_state_dynamics.hpp>

#include <sdm/world/mdp.hpp>
#include <sdm/world/mmdp.hpp>
#include <sdm/world/solvable_by_mdp.hpp>
#include <sdm/world/serialized_mmdp.hpp>

#include <sdm/utils/value_function/tabular_value_function.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    auto state_0 = std::make_shared<DiscreteState>(0);
    auto state_1 = std::make_shared<DiscreteState>(1);
    auto state_2 = std::make_shared<DiscreteState>(2);
    auto state_3 = std::make_shared<DiscreteState>(3);

    auto action_0 = std::make_shared<DiscreteAction>(0);
    auto action_1 = std::make_shared<DiscreteAction>(1);
    auto action_2 = std::make_shared<DiscreteAction>(2);
    auto action_3 = std::make_shared<DiscreteAction>(3);
    
    auto serial_state = std::make_shared<SerializedState>(state_0,Joint<std::shared_ptr<Action>>(std::vector<std::shared_ptr<Action>>({action_2})));

    std::cout<<serial_state->str()<<std::endl;
    std::cout<<"n_agent "<<serial_state->getCurrentAgentId()<<std::endl;

    for(const auto &action : serial_state->getAction())
    {
        std::cout<<"action"<<*action<<std::endl;
    }

    auto state_space_serial = std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{serial_state});

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

    auto state_space = std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{state_0, state_1, state_2, state_3});
    std::cout << *state_space << std::endl;

    std::shared_ptr<Space> single_action_space = std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{action_0, action_1, action_2, action_3});
    std::shared_ptr<Space> action_space = std::make_shared<MultiDiscreteSpace>(std::vector<std::shared_ptr<Space>>{single_action_space, single_action_space});

    auto start_distrib = std::make_shared<DiscreteDistribution<std::shared_ptr<State>>>();
    for (const auto &state : *state_space)
    {
        start_distrib->setProbability(std::static_pointer_cast<State>(state), 1. / state_space->getNumItems());
    }

    auto rew = std::make_shared<TabularReward>();

    double r = 0;
    for (const auto &state : *state_space) // = state_space->begin(); state != state_space->end(); state = state_space->next())
    {
        r += 1;
        for (const auto &action : *action_space) //= action_space->begin(); action != action_space->end(); action = action_space->next())
        {
            r += 1;
            rew->setReward(std::static_pointer_cast<State>(state), std::static_pointer_cast<Action>(action), r);
        }
    }

    // for (const auto &state : *state_space) // = state_space->begin(); state != state_space->end(); state = state_space->next())
    // {
    //     for (const auto &action : *action_space) //= action_space->begin(); action != action_space->end(); action = action_space->next())
    //     {
    //         std::cout << "Reward(" << *state << ", " << *action << ") = " << rew->getReward(std::static_pointer_cast<State>(state), std::static_pointer_cast<Action>(action)) << std::endl;
    //     }
    // }

    std::cout << *rew << std::endl;

    auto dynamics = std::make_shared<TabularStateDynamics>();

    double proba = 1. / state_space->getNumItems();

    for (const auto &state : *state_space) // = state_space->begin(); state != state_space->end(); state = state_space->next())
    {
        for (const auto &action : *action_space) //= action_space->begin(); action != action_space->end(); action = action_space->next())
        {
            for (const auto &next_state : *state_space) // = state_space->begin(); state != state_space->end(); state = state_space->next())
            {
                dynamics->setTransitionProbability(std::static_pointer_cast<State>(state), std::static_pointer_cast<Action>(action), std::static_pointer_cast<State>(next_state), proba);
            }
        }
    }

    // for (const auto &state : *state_space)
    // {
    //     for (const auto &action : *action_space)
    //     {
    //         for (const auto &next_state : *state_space)
    //         {
    //             std::cout << "T(" << *state << ", " << *action << ", " << *next_state << ") = " << dynamics->getTransitionProbability(std::static_pointer_cast<State>(state), std::static_pointer_cast<Action>(action), std::static_pointer_cast<State>(next_state)) << std::endl;
    //         }
    //     }
    // }

    number horizon = 3;

    auto mdp = std::make_shared<MMDP>(horizon, 0.9, state_space, action_space, rew, dynamics, start_distrib);

    auto serial_mmdp = std::make_shared<SerializedMMDP>(mdp);

    // std::cout << "#> NumAgents = " << serial_mmdp->getNumAgents() << std::endl;
    // std::cout << "#> Discount = " << serial_mmdp->getDiscount() << std::endl;
    // std::cout << "#> Reward(2, 2) = " << serial_mmdp->getReward(state_2, action_2) << std::endl;
    // std::cout << "#> StateSpace = " << *serial_mmdp->getStateSpace() << std::endl;
    // std::cout << "#> ActionSpace = " << *serial_mmdp->getActionSpace() << std::endl;

    // std::cout << "#> Reachable from 0, 2 :" << std::endl;
    // for (const auto &reach : serial_mmdp->getReachableStates(state_0, action_2, 0))
    // {
    //     std::cout << "Reachable --> " << *reach << std::endl;
    // }

    // std::cout << "#> Reachable from 0, 0 :" << std::endl;
    // for (const auto &reach : serial_mmdp->getReachableStates(state_0, action_0, 0))
    // {
    //     std::cout << "Reachable --> " << *reach << std::endl;
    // }

    // std::shared_ptr<SolvableByHSVI> hsvi_serial_mmdp = std::make_shared<SolvableByMDP>(serial_mmdp);

    // auto lb = std::make_shared<MappedValueFunction>(hsvi_serial_mmdp, horizon, -1000);
    // auto ub = std::make_shared<MappedValueFunction>(hsvi_serial_mmdp, horizon, 1000);

    // auto algo = std::make_shared<HSVI>(hsvi_serial_mmdp, lb, ub, horizon, 0.01);
    // algo->do_initialize();
    // std::cout << *algo->getLowerBound() << std::endl;
    // std::cout << *algo->getUpperBound() << std::endl;
    // algo->do_solve();
    // std::cout << *algo->getLowerBound() << std::endl;
    // std::cout << *algo->getUpperBound() << std::endl;

} // END main
