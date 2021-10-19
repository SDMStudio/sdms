

#include <sdm/utils/value_function/action_selection/constraint_programming_selection.hpp>

namespace sdm
{

    ConstraintProgrammingSelection::ConstraintProgrammingSelection()
    {
    }

    ConstraintProgrammingSelection::ConstraintProgrammingSelection(const std::shared_ptr<SolvableByHSVI> &world) : ActionSelectionBase(world)
    {
    }

    Pair<std::shared_ptr<Action>, double> ConstraintProgrammingSelection::getGreedyActionAndValue(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, number t)
    {
        std::shared_ptr<Action> max_decision_rule;
        double max_value = -std::numeric_limits<double>::max();

        // Go over all hyperplan in the Support
        for (const auto &hyperplan : vf->getSupport(t + 1))
        {
            this->tmp_representation = hyperplan->toBelief();
            auto pair_action_value = this->createAndSolveCP(vf, state, t);

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