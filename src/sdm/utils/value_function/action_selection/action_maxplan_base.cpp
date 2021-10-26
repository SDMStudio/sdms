

#include <sdm/utils/value_function/action_selection/action_maxplan_base.hpp>
#include <sdm/utils/value_function/value_function_interface.hpp>
#include <sdm/utils/value_function/pwlc_value_function_interface.hpp>

namespace sdm
{

    MaxPlanSelectionBase::MaxPlanSelectionBase()
    {
    }

    MaxPlanSelectionBase::MaxPlanSelectionBase(const std::shared_ptr<SolvableByHSVI> &world) : ActionSelectionBase(world)
    {
    }

    Pair<std::shared_ptr<Action>, double> MaxPlanSelectionBase::getGreedyActionAndValue(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, number t)
    {
        // Cast the generic value function into a piece-wise linear convex value function.
        auto value_function = std::dynamic_pointer_cast<PWLCValueFunctionInterface>(vf);

        // Instanciate return variables
        std::shared_ptr<Action> max_decision_rule;
        double max_value = -std::numeric_limits<double>::max();

        // Go over all hyperplan in the Support
        for (const auto &hyperplan : value_function->getHyperplanesAt(t + 1))
        {
            this->tmp_representation = hyperplan->toBelief();
            
            // Compute the greedy action and value for a given hyperplan
            auto pair_action_value = this->computeGreedyActionAndValue(value_function, state, t);

            // Select the Best Action
            if (pair_action_value.second > max_value)
            {
                max_decision_rule = pair_action_value.first;
                max_value = pair_action_value.second;
            }
        }
        return {max_decision_rule, max_value};
    }
} // namespace sdm