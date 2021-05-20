
#pragma once
#include <set>

#include <sdm/core/state/serialized_state.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/value_function/initializer.hpp>
#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>

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
        TState_t ctype = COMPRESSED;
        TypeSawtoothLinearProgram csawtooth_lp_ = PLAIN_SAWTOOTH_LINER_PROGRAMMING;

        /**
         * @brief Frequency before prunning
         * 
         */
        number freq_prune_;

        /**
         * @brief The last time the prunning took place
         * 
         */
        number last_prunning = 0;

        /**
         * @brief epsilon value used for the prunning
         * 
         */
        double epsilon_prunning;

        TState_t getTStateType();
        void setTStateType(const TState_t &);

        TypeSawtoothLinearProgram getSawtoothType();
        void setSawtoothType(const TypeSawtoothLinearProgram &);

    public:
        SawtoothValueFunction();

        /**
         * @brief Construct a new Sawtooth Value Function object
         * 
         * @param std::shared_ptr<SolvableByHSVI<TState, TAction>> : problem 
         * @param number : horizon 
         * @param std::shared_ptr<Initializer<TState, TAction>> : initializer 
         * @param number frequency of the pruning 
         * @param double : espilon prunning 
         */
        SawtoothValueFunction(std::shared_ptr<SolvableByHSVI<TState, TAction>> , number , std::shared_ptr<Initializer<TState, TAction>>, number = 10, double = 0.1);
        SawtoothValueFunction(std::shared_ptr<SolvableByHSVI<TState, TAction>> , number  = 0, TValue  = 0., number = 10, double = 0.1);

        /**
         * @brief Evaluate the value at a state.
         * 
         * @param state the state where we want to evaluate the function
         * @return the value
         */
        TValue getValueAt(const TState &state, number t = 0);

        void updateValueAt(const TState &state, number t);

        virtual TValue getBackup(const TState &state, number t);


        std::pair<TValue, TState> getMaxAt(const TState &state, number t);

        /**
         * @brief Point-wise pruning
         * 
         */
        void prune(number t = 0);

        bool is_dominated(const TState &ostate, double value, number t);
    };
}
#include <sdm/utils/value_function/sawtooth_vf.tpp>
