#include <sdm/utils/value_function/evaluate_vf/evaluate_maxplan.hpp>
#include <sdm/core/state/belief_state.hpp>
#include <sdm/utils/value_function/value_function.hpp>

namespace sdm
{
    Pair<std::shared_ptr<State>,double> EvaluateMaxplanInterface::evaluate(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t)
    {
        double current, max = -std::numeric_limits<double>::max();
        std::shared_ptr<BeliefInterface> alpha_vector;

        auto belief_state = state->toBelief();

        for (const auto &plan : vf->getSupport(t))
        {
            auto belief_plan = plan->toBelief();

            current = belief_state->operator^(belief_plan);

            if (max < current)
            {
                max = current;
                alpha_vector = belief_plan;
            }
        }
        return {alpha_vector,max};
    }
}