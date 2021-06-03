#include <iostream>

#include <memory>
#include <sdm/exception.hpp>

#include <sdm/algorithms/hsvi.hpp>

#include <sdm/core/state/base_state.hpp>
#include <sdm/core/action/base_action.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/distribution.hpp>
#include <sdm/core/reward/tabular_reward.hpp>
#include <sdm/core/dynamics/tabular_state_dynamics.hpp>

#include <sdm/world/base/base_mdp.hpp>
#include <sdm/world/mdp.hpp>

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

    // auto test = MultiDiscreteSpace<number>();
    auto test = MultiDiscreteSpace<std::shared_ptr<Action>>();
    // auto test = MultiDiscreteSpace<std::shared_ptr<Observation>>();

    std::cout << *state_0 << std::endl;
    std::cout << *action_3 << std::endl;

    auto state_space = std::make_shared<DiscreteSpace<std::shared_ptr<State>>>(std::vector<std::shared_ptr<State>>{state_0, state_1, state_2, state_3});
    std::cout << *state_space << std::endl;

    auto action_space = std::make_shared<DiscreteSpace<std::shared_ptr<Action>>>(std::vector<std::shared_ptr<Action>>{action_0, action_1, action_2, action_3});
    std::cout << *action_space << std::endl;

    auto start_distrib = std::make_shared<DiscreteDistribution<std::shared_ptr<State>>>();
    for (const auto &state : state_space->getAll())
    {
        start_distrib->setProbability(state, 1. / state_space->getNumItems());
    }

    std::cout << "proba state 3 = " << start_distrib->getProbability(state_3) << std::endl;

    std::cout << "Sample -> " << start_distrib->sample() << std::endl;
    std::cout << "Sample -> " << start_distrib->sample() << std::endl;
    std::cout << "Sample -> " << start_distrib->sample() << std::endl;
    std::cout << "Sample -> " << start_distrib->sample() << std::endl;

    auto rew = std::make_shared<TabularReward>();

    double r = 0;
    for (const auto &state : state_space->getAll()) // = state_space->begin(); state != state_space->end(); state = state_space->next())
    {
        r += 1;
        for (const auto &action : action_space->getAll()) //= action_space->begin(); action != action_space->end(); action = action_space->next())
        {
            r += 1;
            rew->setReward(state, action, r);
        }
    }

    for (const auto &state : state_space->getAll()) // = state_space->begin(); state != state_space->end(); state = state_space->next())
    {
        for (const auto &action : action_space->getAll()) //= action_space->begin(); action != action_space->end(); action = action_space->next())
        {
            std::cout << "Reward(" << *state << ", " << *action << ") = " << rew->getReward(state, action) << std::endl;
        }
    }

    std::cout << *rew << std::endl;

    auto dynamics = std::make_shared<TabularStateDynamics>();

    double proba = 1. / state_space->getNumItems();

    for (const auto &state : state_space->getAll()) // = state_space->begin(); state != state_space->end(); state = state_space->next())
    {
        for (const auto &action : action_space->getAll()) //= action_space->begin(); action != action_space->end(); action = action_space->next())
        {
            for (const auto &next_state : state_space->getAll()) // = state_space->begin(); state != state_space->end(); state = state_space->next())
            {
                dynamics->setTransitionProbability(state, action, next_state, proba);
            }
        }
    }

    for (const auto &state : state_space->getAll())
    {
        for (const auto &action : action_space->getAll())
        {
            for (const auto &next_state : state_space->getAll())
            {
                std::cout << "T(" << *state << ", " << *action << ", " << *next_state << ") = " << dynamics->getTransitionProbability(state, action, next_state) << std::endl;
            }
        }
    }

    auto mdp = std::make_shared<BaseMDP>(1, 0.9, state_space, action_space, rew, dynamics, start_distrib);

    std::cout << "#> NumAgents = " << mdp->getNumAgents() << std::endl;
    std::cout << "#> Discount = " << mdp->getDiscount() << std::endl;
    std::cout << "#> Reward(2, 2) = " << mdp->getReward(state_2, action_2) << std::endl;
    std::cout << "#> StateSpace = " << *mdp->getStateSpace() << std::endl;
    std::cout << "#> ActionSpace = " << *mdp->getActionSpace() << std::endl;

    std::cout << "#> Reachable from 0, 2 :" << std::endl;
    for (const auto &reach : mdp->getReachableStates(state_0, action_2, 0))
    {
        std::cout << "Reachable --> " << *reach << std::endl;
    }

    std::cout << "#> Reachable from 0, 0 :" << std::endl;
    for (const auto &reach : mdp->getReachableStates(state_0, action_0, 0))
    {
        std::cout << "Reachable --> " << *reach << std::endl;
    }


    std::shared_ptr<SolvableByHSVI> hsvi_mdp = std::make_shared<MDP>(mdp);


    auto lb = std::make_shared<MappedValueFunction>(hsvi_mdp, 3, -1000);
    auto ub = std::make_shared<MappedValueFunction>(hsvi_mdp, 3, 1000);

    auto algo = std::make_shared<HSVI>(hsvi_mdp, lb, ub, 3, 0.01);
    algo->do_initialize();
    std::cout << *algo->getLowerBound() << std::endl;
    std::cout << *algo->getUpperBound() << std::endl;
    algo->do_solve();
    std::cout << *algo->getLowerBound() << std::endl;
    std::cout << *algo->getUpperBound() << std::endl;

} // END main
