#include <sdm/utils/value_function/action_selection/exhaustive_action_selection.hpp>

namespace sdm
{
    ExhaustiveActionSelection::ExhaustiveActionSelection() {}

    ExhaustiveActionSelection::ExhaustiveActionSelection(const std::shared_ptr<SolvableByHSVI> &world) : ActionSelectionBase(world) {}

    Pair<std::shared_ptr<Action>, double> ExhaustiveActionSelection::getGreedyActionAndValue(const std::shared_ptr<ValueFunction> &value_function, const std::shared_ptr<State> &state, number t)
    {
        std::shared_ptr<Action> best_action;
        double max = -std::numeric_limits<double>::max(), tmp;

        auto action_space = this->getWorld()->getActionSpaceAt(state, t);

        //Go over all actions
        for (const auto &action : *action_space)
        {
            //Determine the value of the backup for a precise action
            auto casted_action = action->toAction();
            if (max < (tmp = value_function->getQValueAt(state, casted_action, t)))
            {
                best_action = casted_action;
                max = tmp;
            }
        }
        //Return the best action and the value associated
        return {best_action, max};
    }
}