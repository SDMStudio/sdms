#include <sdm/utils/value_function/evaluate_vf/evaluate_sawtooth.hpp>
#include <sdm/core/state/belief_state.hpp>
#include <sdm/utils/value_function/value_function.hpp>

namespace sdm
{
    Pair<std::shared_ptr<State>,double> EvaluateSawtoothInterface::evaluate(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t)
    {
        assert(vf->getInitFunction() != nullptr);
        assert(state->getTypeState() != TypeState::STATE);

        auto belief_state = state->toBelief();

        double min_ext = 0;
        double v_ub_state = vf->getInitFunction()->operator()(state, t);

        std::shared_ptr<State> argmin_ = state;

        for (const auto &element : vf->getSupport(t))
        {
            auto element_belief_state = element->toBelief();

            double v_kappa = vf->getValueAt(element, t);
            double v_ub_kappa = vf->getInitFunction()->operator()(element, t);

            double phi = 1.0;
            
            for (auto &state_element : element_belief_state->getStates())
            {
                double v_int = (belief_state->getProbability(state_element) / element_belief_state->getProbability(state_element));
                if (v_int < phi)
                {
                    phi = v_int;
                }
            }

            double min_int = phi * (v_kappa - v_ub_kappa);
            if (min_int < min_ext)
            {
                min_ext = min_int;
                argmin_ = element_belief_state;
            }
        }
        return std::make_pair(argmin_,v_ub_state + min_ext);
    }
}