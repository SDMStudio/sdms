#include <sdm/utils/value_function/tabular_value_function.hpp>

namespace sdm
{
    SawtoothValueFunction::SawtoothValueFunction() {}

    SawtoothValueFunction::SawtoothValueFunction(std::shared_ptr<SolvableByHSVI> problem, number horizon, std::shared_ptr<Initializer> initializer, number freq_prune, double epsilon)
        : MappedValueFunction(problem, horizon, initializer), freq_prune_(freq_prune), epsilon_prunning(epsilon)
    {
    }

    SawtoothValueFunction::SawtoothValueFunction(std::shared_ptr<SolvableByHSVI> problem, number horizon, double default_value, number freq_prune, double epsilon)
        : SawtoothValueFunction(problem, horizon, std::make_shared<ValueInitializer>(default_value), freq_prune, epsilon)
    {
    }

    double SawtoothValueFunction::getValueAt(const std::shared_ptr<State> &state, number t)
    {
        if (this->isInfiniteHorizon())
        {
            return this->getMaxAt(state, 0).first;
        }
        else
        {
            bool already_exist = false;
            for (auto iter = this->representation[t].begin(); iter != this->representation[t].end(); iter++)
            {
                if (iter->first == state)
                {
                    already_exist = true;
                }
            }

            return (already_exist) ? this->representation[t].at(state) : this->getMaxAt(state, t).first;
        }
    }

    void SawtoothValueFunction::updateValueAt(const std::shared_ptr<State> &state, number t)
    {
        MappedValueFunction::updateValueAt(state, t, this->getBackup(state, t));

        if (this->last_prunning == this->freq_prune_)
        {
            for (number time = 0; time < this->getHorizon(); time++)
            {
                this->prune(time);
            }
            this->last_prunning = 0;
        }
        this->last_prunning++;
    }

    std::pair<double, std::shared_ptr<State>> SawtoothValueFunction::getMaxAt(const std::shared_ptr<State> &state, number t)
    {
        // Attention pour le moment, ça risque de ne pas fonctionner, il faut créer une classes State plus précise


        // assert(this->getInitFunction() != nullptr);

        // double min_ext = 0;
        // double v_ub_state = this->getInitFunction()->operator()(state, t);

        // std::shared_ptr<State> argmin_ = state;

        // for (const auto &pair_ostate_value : this->representation[t])
        // {
        //     auto ostate = pair_ostate_value.first;
        //     double v_kappa = pair_ostate_value.second;
        //     double v_ub_kappa = this->getInitFunction()->operator()(ostate, t);

        //     double phi = 1.0;
        //     for (auto &pair_hidden_state_AND_joint_history_AND_probability : ostate)
        //     {
        //         double v_int = (state.at(pair_hidden_state_AND_joint_history_AND_probability.first) / pair_hidden_state_AND_joint_history_AND_probability.second);
        //         if (v_int < phi)
        //         {
        //             phi = v_int;
        //         }
        //     }

        //     double min_int = phi * (v_kappa - v_ub_kappa);
        //     if (min_int < min_ext)
        //     {
        //         min_ext = min_int;
        //         argmin_ = ostate;
        //     }
        // }
        // return std::make_pair(v_ub_state + min_ext, argmin_);
    }

    void SawtoothValueFunction::prune(number t)
    {
        std::vector<std::shared_ptr<State>> to_delete;

        for (auto iter = this->representation[t].begin(); iter != this->representation[t].end(); iter++)
        {
            if (this->is_dominated(iter->first, iter->second, t))
            {
                to_delete.push_back(iter->first);
            }
        }

        for (const auto &i : to_delete)
        {
            this->representation[t].erase(i);
        }
    }

    bool SawtoothValueFunction::is_dominated(const std::shared_ptr<State> &state, double value, number t)
    {
        auto pair_witness_ostate = this->getMaxAt(state, t);

        if (pair_witness_ostate.second == state)
        {
            return false;
        }
        else
        {
            return (pair_witness_ostate.first <= value + this->epsilon_prunning);
        }
    }
}