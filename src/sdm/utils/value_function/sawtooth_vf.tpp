#include <sdm/utils/value_function/tabular_value_function.hpp>

namespace sdm
{
    template <typename TState, typename TAction, typename TValue>
    SawtoothValueFunction<TState, TAction, TValue>::SawtoothValueFunction() {}

    template <typename TState, typename TAction, typename TValue>
    SawtoothValueFunction<TState, TAction, TValue>::SawtoothValueFunction(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, number horizon, std::shared_ptr<Initializer<TState, TAction>> initializer, number freq_prune, double epsilon)
        : MappedValueFunction<TState, TAction, TValue>(problem, horizon, initializer), freq_prune_(freq_prune), epsilon_prunning(epsilon)
    {
    }

    template <typename TState, typename TAction, typename TValue>
    SawtoothValueFunction<TState, TAction, TValue>::SawtoothValueFunction(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, number horizon, TValue default_value, number freq_prune, double epsilon)
        : SawtoothValueFunction(problem, horizon, std::make_shared<ValueInitializer<TState, TAction>>(default_value), freq_prune,epsilon)
    {
    }

    template <typename TState, typename TAction, typename TValue>
    TValue SawtoothValueFunction<TState, TAction, TValue>::getValueAt(const TState &state, number t)
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

    template <>
    double SawtoothValueFunction<number, number, double>::getValueAt(const number &, number)
    {
        throw sdm::exception::Exception("SawtoothValueFunction cannot be used for State = number.");
    }

    template <>
    double SawtoothValueFunction<SerializedState, number, double>::getValueAt(const SerializedState &, number)
    {
        throw sdm::exception::Exception("SawtoothValueFunction cannot be used for State = SerializedState.");
    }

    template <>
    double SawtoothValueFunction<BeliefState, number, double>::getValueAt(const BeliefState &, number)
    {
        throw sdm::exception::Exception("SawtoothValueFunction cannot be used for State = BeliefState.");
    }

    template <>
    double SawtoothValueFunction<SerializedBeliefState, number, double>::getValueAt(const SerializedBeliefState &, number)
    {
        throw sdm::exception::Exception("SawtoothValueFunction cannot be used for State = SerializedBeliefState.");
    }

    template <typename TState, typename TAction, typename TValue>
    void SawtoothValueFunction<TState, TAction, TValue>::updateValueAt(const TState &state, number t, const TValue& value)
    {
        MappedValueFunction<TState, TAction, TValue>::updateValueAt(state, t, value);

        if (this->last_prunning == this->freq_prune_)
        {
            for(number time =0; time<this->getHorizon();time++)
            {
                this->prune(time);
            }
            this->last_prunning = 0;
        }
        this->last_prunning ++;
    }

    template <typename TState, typename TAction, typename TValue>
    void SawtoothValueFunction<TState, TAction, TValue>::updateValueAt(const TState &state, number t)
    {
        SawtoothValueFunction<TState, TAction, TValue>::updateValueAt(state, t, this->getBackup(state, t));
    }

    template <>
    void SawtoothValueFunction<number, number, double>::updateValueAt(const number &, number)
    {
        throw sdm::exception::Exception("SawtoothValueFunction cannot be used for State = number.");
    }

    template <>
    void SawtoothValueFunction<SerializedState, number, double>::updateValueAt(const SerializedState &, number)
    {
        throw sdm::exception::Exception("SawtoothValueFunction cannot be used for State = SerializedState.");
    }

    template <>
    void SawtoothValueFunction<BeliefState, number, double>::updateValueAt(const BeliefState &, number)
    {
        throw sdm::exception::Exception("SawtoothValueFunction cannot be used for State = BeliefState.");
    }

    template <>
    void SawtoothValueFunction<SerializedBeliefState, number, double>::updateValueAt(const SerializedBeliefState &, number)
    {
        throw sdm::exception::Exception("SawtoothValueFunction cannot be used for State = SerializedBeliefState.");
    }

    template <typename TState, typename TAction, typename TValue>
    TValue SawtoothValueFunction<TState, TAction, TValue>::getBackup(const TState &state, number t)
    {
        return  this->getBackupOperator().backup(this->getptr(), state, t);
    }


    template <typename TState, typename TAction, typename TValue>
    std::pair<TValue, TState> SawtoothValueFunction<TState, TAction, TValue>::getMaxAt(const TState &state, number t)
    {
        assert(this->getInitFunction() != nullptr);

        double min_ext = 0;
        double v_ub_state = this->getInitFunction()->operator()(state, t);

        TState argmin_ = state;

        for (const auto &pair_ostate_value : this->representation[t])
        {
            auto ostate = pair_ostate_value.first;
            double v_kappa = pair_ostate_value.second;
            double v_ub_kappa = this->getInitFunction()->operator()(ostate, t);

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

    template <typename TState, typename TAction, typename TValue>
    void SawtoothValueFunction<TState, TAction, TValue>::prune(number t)
    {
        std::vector<TState> to_delete;

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

    template <typename TState, typename TAction, typename TValue>
    bool SawtoothValueFunction<TState, TAction, TValue>::is_dominated(const TState &state, double value, number t)
    {
        // TState state_;
        // switch (this->ctype)
        // {
        //     case TState_t::FULLY_UNCOMPRESSED:
        //         state_ = *state.getFullyUncompressedOccupancy();
        //         break;
        //     case TState_t::ONE_STEP_UNCOMPRESSED:
        //         state_ = *state.getOneStepUncompressedOccupancy();
        //         break;
        //     default:
        //         state_ = state;
        //         break;
        // }

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

    template <>
    std::pair<double, number> SawtoothValueFunction<number, number, double>::getMaxAt(const number &, number)
    {
        throw sdm::exception::Exception("SawtoothValueFunction cannot be used for State = number.");
    }

    template <>
    std::pair<double, SerializedState> SawtoothValueFunction<SerializedState, number, double>::getMaxAt(const SerializedState &, number)
    {
        throw sdm::exception::Exception("SawtoothValueFunction cannot be used for State = SerializedState.");
    }
}