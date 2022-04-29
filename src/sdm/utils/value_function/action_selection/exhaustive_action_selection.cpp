#include <sdm/utils/value_function/action_selection/exhaustive_action_selection.hpp>
#include <sdm/utils/value_function/value_function_interface.hpp>

namespace sdm
{
    ExhaustiveActionSelection::ExhaustiveActionSelection() {}

    ExhaustiveActionSelection::ExhaustiveActionSelection(const std::shared_ptr<SolvableByDP> &world, Config config) : ActionSelectionBase(world) {}

    Pair<std::shared_ptr<Action>, double> ExhaustiveActionSelection::getGreedyActionAndValue(const std::shared_ptr<ValueFunctionInterface> &value_function, const std::shared_ptr<State> &state, number t)
    {

        std::shared_ptr<Action> best_action;
        double max = -std::numeric_limits<double>::max(), tmp;

        // Go over all actions
        auto action_space = this->getWorld()->getActionSpaceAt(state, t);
        auto action_end_iter = action_space->end();
        for (auto action_iter = action_space->begin(); !action_iter->equal(action_end_iter); action_iter = action_iter->next())
        {
            auto action = action_iter->getCurrent();

            // Determine the value of the backup for a precise action
            if (max < (tmp = value_function->getQValueAt(state, action, t)))
            {
                best_action = action;
                max = tmp;
            }
        }
        // Return the best action and the value associated
        return {best_action, max};
    }
}