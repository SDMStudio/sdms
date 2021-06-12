#include <sdm/utils/value_function/backup/sawtooth_backup.hpp>

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

        double min_ext = 0;
        double v_ub_state = vf->getInitFunction()->operator()(state, t);

        std::shared_ptr<State> argmin_ = state;

        for (const auto &pair_ostate_value : vf->getSupport(t))
        {
            // A ce niveau, il manque un cast pour parcourir les états.
            // auto occupancy_state = std::static_pointer_cast<State>(action);

            auto ostate = pair_ostate_value;
            double v_kappa = vf->getValueAt(pair_ostate_value, t);
            double v_ub_kappa = vf->getInitFunction()->operator()(ostate, t);

            double phi = 1.0;
            for (auto &pair_hidden_state_AND_joint_history_AND_probability : ostate)
            {
                double v_int = (state.at(pair_hidden_state_AND_joint_history_AND_probability.first) / pair_hidden_state_AND_joint_history_AND_probability.second);
                if (v_int < phi)
                {
                    phi = v_int;
                }
            }

            double min_int = phi * (v_kappa - v_ub_kappa);
            if (min_int < min_ext)
            {
                min_ext = min_int;
                argmin_ = ostate;
            }
        }
        return std::make_pair(v_ub_state + min_ext, argmin_);
    }

    double SawtoothBackup::getBackup(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t)
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