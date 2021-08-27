#include <iostream>

#include <memory>
#include <sdm/exception.hpp>
#include <sdm/core/state/base_state.hpp>
#include <sdm/core/action/base_action.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/reward/tabular_reward.hpp>

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

    auto state_space = std::make_shared<DiscreteSpace>({state_0, state_1, state_2, state_3});
    auto action_space = std::make_shared<DiscreteSpace>({action_0, action_1, action_2, action_3});

    std::shared_ptr<TabularReward> rew;

    double r = 0;
    for (const auto &state = state_space->begin(); state != state_space->end(); state = state_space->next())
    {
        r += 1;
        for (const auto &action = action_space->begin(); action != action_space->end(); action = action_space->next())
        {
            r += 1;
            rew->setReward(state, action, r);
        }
    }

    for (const auto &state = state_space->begin(); state != state_space->end(); state = state_space->next())
    {
        for (const auto &action = action_space->begin(); action != action_space->end(); action = action_space->next())
        {
            std::cout << "Reward(" << *state << ", " << *action << ") = " << rew->getReward(state, action) << std::endl;
        }
    }

} // END main
