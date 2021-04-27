#include <sdm/utils/value_function/tabular_value_function.hpp>

namespace sdm
{
    template <typename TState, typename TAction, typename TValue>
    SawtoothValueFunction<TState, TAction, TValue>::SawtoothValueFunction() {}

    template <typename TState, typename TAction, typename TValue>
    SawtoothValueFunction<TState, TAction, TValue>::SawtoothValueFunction(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, number horizon, std::shared_ptr<Initializer<TState, TAction>> initializer) : MappedValueFunction<TState, TAction, TValue>(problem, horizon, initializer)
    {

    }

    template <typename TState, typename TAction, typename TValue>
    SawtoothValueFunction<TState, TAction, TValue>::SawtoothValueFunction(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, number horizon, TValue default_value) : SawtoothValueFunction(problem, horizon, std::make_shared<ValueInitializer<TState, TAction>>(default_value))
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
            if (t >= this->getHorizon())
            {
                return 0;
            }

            bool already_exist = false;
            for (auto iter = this->representation[t].begin(); iter != this->representation[t].end(); iter++)
            {  
                if(iter->first == state)
                {
                    already_exist = true;
                }
            }

            return (already_exist) ?this->representation[t].at(state) : this->getMaxAt(state, t).first;
            // if (*std::find(this->getSupport(t).begin(),this->getSupport(t).end(),state) == state)
            // {
            //     //return this->representation[t].at(state);
            // }else
            // {
            //     return this->getMaxAt(state, t).first; 
            // }
            //return 0;
            //return (t >= this->getHorizon()) ? 0 : this->getMaxAt(state, t).first;
        }
    }

    template <typename TState, typename TAction, typename TValue>
    void SawtoothValueFunction<TState, TAction, TValue>::updateValueAt(const TState &state, number t)
    {
        MappedValueFunction<TState, TAction, TValue>::updateValueAt(state, t, this->getBackupOperator().backup(this, state, t));
        this->prune(t);
    }

    template <typename TState, typename TAction, typename TValue>
    std::pair<TValue, TState> SawtoothValueFunction<TState, TAction, TValue>::getMaxAt(const TState &state, number t)
    {
        assert(this->getInitFunction() != nullptr); 

        double v_ub_state =this->getInitFunction()->operator()(state, t);
        double min_ext = 0;

        TState argmin_ = state;
        for (const TState &ostate : this->getSupport(t))
        {
            double v_kappa = this->getValueAt(ostate, t); // Problème boucle infinie 
            double v_ub_kappa = this->getInitFunction()->operator()(ostate, t);
            double phi = std::numeric_limits<double>::max();
            for (auto &x : ostate)
            {
                double v_int = (state.at(x.first) / x.second);
                if (v_int < phi)
                {
                    phi = v_int;
                }
            }
            double min_int = phi * (v_kappa - v_ub_kappa);
            if (min_int < min_ext)
            {
                min_ext = min_int;
                argmin_ = ostate; // to verify
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

        for(const auto &i : to_delete)
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
}