#include <sdm/utils/value_function/tabular_value_function.hpp>

namespace sdm
{
    template <typename TState, typename TAction, typename TValue>
    SawtoothValueFunction<TState, TAction, TValue>::SawtoothValueFunction() {}

    template <typename TState, typename TAction, typename TValue>
    SawtoothValueFunction<TState, TAction, TValue>::SawtoothValueFunction(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, number horizon, std::shared_ptr<Initializer<TState, TAction>> initializer)
        : MappedValueFunction<TState, TAction, TValue>(problem, horizon, initializer)
    {
    }

    template <typename TState, typename TAction, typename TValue>
    SawtoothValueFunction<TState, TAction, TValue>::SawtoothValueFunction(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, number horizon, TValue default_value)
        : SawtoothValueFunction(problem, horizon, std::make_shared<ValueInitializer<TState, TAction>>(default_value))
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

    template <typename TState, typename TAction, typename TValue>
    void SawtoothValueFunction<TState, TAction, TValue>::updateValueAt(const TState &state, number t)
    {
        MappedValueFunction<TState, TAction, TValue>::updateValueAt(state, t, this->getBackupOperator().backup(this->getptr(), state, t));
        // this->prune(t);
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
    bool SawtoothValueFunction<TState, TAction, TValue>::is_dominated(const TState &ostate, double value, number t)
    {
        auto pair_witness_ostate = this->getMaxAt(ostate, t);

        if (pair_witness_ostate.second == ostate)
        {
            return false;
        }
        else
        {
            return (pair_witness_ostate.first <= value);
        }
    }

    template <typename TState, typename TAction, typename TValue>
    TState_t SawtoothValueFunction<TState, TAction, TValue>::getTStateType()
    {
        return this->ctype;
    }

    template <typename TState, typename TAction, typename TValue>
    void SawtoothValueFunction<TState, TAction, TValue>::setTStateType(const TState_t &ctype)
    {
        this->ctype = ctype;
    }
}