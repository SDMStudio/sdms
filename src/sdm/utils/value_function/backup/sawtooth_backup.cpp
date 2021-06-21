#include <sdm/utils/value_function/backup/sawtooth_backup.hpp>
#include <sdm/core/state/interface/belief_interface.hpp>

namespace sdm
{
    SawtoothBackup::SawtoothBackup() {}

    SawtoothBackup::SawtoothBackup(std::shared_ptr<SolvableByHSVI> world)
    {
        this->world_ = world;
    }

    std::pair<double, std::shared_ptr<State>> SawtoothBackup::getMaxAt(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t)
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
        return std::make_pair(v_ub_state + min_ext, argmin_);
    }

    double SawtoothBackup::backup(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t)
    {
        return this->getMaxAt(vf, state, t).first;
    }

    std::shared_ptr<Action> SawtoothBackup::getBestAction(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t)
    {
        std::shared_ptr<Action> best_action;
        double max = -std::numeric_limits<double>::max(), tmp;

        for (const auto &action : *this->world_->getActionSpaceAt(state, t))
        {
            auto casted_action = std::static_pointer_cast<Action>(action);
            if (max < (tmp = this->getQValueAt(vf, state, casted_action, t)))
            {
                best_action = casted_action;
                max = tmp;
            }
        }
        return best_action;
    }

}