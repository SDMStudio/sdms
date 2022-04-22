#include <iostream>
#include <sdm/types.hpp>
#include <sdm/core/action/base_action.hpp>
#include <sdm/core/state/base_state.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/space/iterator/super_iterator.hpp>
#include <sdm/core/space/iterator/combination_iterator.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    auto s0 = std::make_shared<StringState>("s0"), s1 = std::make_shared<StringState>("s1"), s2 = std::make_shared<StringState>("s2");
    std::vector<std::shared_ptr<State>> states = {s0, s1, s2};

    auto up = std::make_shared<StringAction>("up"), down = std::make_shared<StringAction>("down"), fire = std::make_shared<StringAction>("fire");
    std::vector<std::vector<std::shared_ptr<Action>>> actions = {{up, fire, fire, down}, {up, down, fire}};

    auto state_sp = std::make_shared<DiscreteStateSpace>(states);
    auto action_sp = std::make_shared<MultiDiscreteActionSpace>(actions);

    for (const auto &state : *state_sp)
    {
        std::cout << "s=" << *state << std::endl;
    }

    for (const auto &action : *action_sp)
    {
        std::cout << "a=" << *action << std::endl;
    }

    // auto iter_state_sp1 = state_sp1->getIterator();

    // for (iter_state_sp1->begin(); iter_state_sp1->end(); iter_state_sp1->next())
    // {
    //     auto val = iter_state_sp1->getCurrent();
    // }
}
