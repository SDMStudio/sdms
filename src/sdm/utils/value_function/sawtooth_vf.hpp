
#pragma once
#include <set>

#include <sdm/core/state/serialized_state.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/value_function/initializer.hpp>
#include <sdm/utils/value_function/value_function.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    /**
     * @brief 
     * 
     * @tparam TState type of hyperplan representation. Must implement sdm::VectorImpl interface.
     * @tparam TValue value type (default : double)
     */
    template <typename TState, typename TAction, typename TValue = double>
    class SawtoothValueFunction : public MappedValueFunction<TState, TAction, TValue>
    {
    protected:
    public:
        SawtoothValueFunction();
        SawtoothValueFunction(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, int horizon, std::shared_ptr<Initializer<TState, TAction>> initializer);
        SawtoothValueFunction(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, int horizon = 0, TValue default_value = 0.);

        /**
         * @brief Evaluate the value at a state.
         * 
         * @param state the state where we want to evaluate the function
         * @return the value
         */
        TValue getValueAt(const TState &state, number t = 0)
        {
            this->getMaxAt(state, t).first;
        }

        void updateValueAt(const TState &state, number t)
        {
            this->updateValueAt(state, t, this->getBackupOperator().backup(this, state, t));
            this->prune(t);
        }

        std::pair<TValue, TState> getMaxAt(const TState &state, number t)
        {

            double v_ub_state = this->getInitFunction()->operator()(state, t);
            double min_ext = 0;

            TState argmin_ = state; // to verify
            for (const TState &ostate : this->getSupport(t))
            {
                double v_kappa = this->getValueAt(ostate, t);
                double v_ub_kappa = this->getInitFunction()->operator()(ostate, t);
                double phi = std::numeric_limits<double>::max();
                for (auto &x : ostate)
                {
                    double v_int = (ostate.at(x.first) / x.second);
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

        /**
         * @brief Point-wise pruning
         * 
         */
        void prune(number t = 0)
        {
            auto iter = this->representation[t].begin();
            for (iter; iter != this->representation[t].end(); iter++)
            {
                if (this->is_dominated(iter->first, iter->second, t))
                {
                    this->representation[t].erase(iter->first);
                    break;
                }
            }
        }

        bool is_dominated(const TState &ostate, double value, number t)
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
    };
}