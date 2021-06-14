
#pragma once
#include <set>

#include <sdm/core/state/serialized_state.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/value_function/initializer/initializer.hpp>
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
     */
    class SawtoothValueFunction : public MappedValueFunction
    {
    protected:
        /**
         * @brief Frequency before prunning.
         */
        number freq_prune_;

        /**
         * @brief The last time the prunning took place.
         * 
         */
        number last_prunning = 0;

        /**
         * @brief epsilon value used for the prunning
         * 
         */
        double epsilon_prunning;

    public:
        SawtoothValueFunction();

        /**
         * @brief Construct a new Sawtooth Value Function object
         * 
         * @param std::shared_ptr<SolvableByHSVI> : problem 
         * @param number : horizon 
         * @param std::shared_ptr<Initializer> : initializer 
         * @param number frequency of the pruning 
         * @param double : espilon prunning 
         */
        SawtoothValueFunction(std::shared_ptr<SolvableByHSVI> , number , std::shared_ptr<Initializer>, number = 10, double = 0.1);
        
        SawtoothValueFunction(std::shared_ptr<SolvableByHSVI> , number  = 0, double  = 0., number = 10, double = 0.1);

        /**
         * @brief Evaluate the value at a state.
         * 
         * @param state the state where we want to evaluate the function
         * @return the value
         */
        double getValueAt(const std::shared_ptr<State> &state, number t = 0);

        void updateValueAt(const std::shared_ptr<State> &state, number t);

        virtual double getBackup(const std::shared_ptr<State> &state, number t);

        std::pair<double, std::shared_ptr<State>> getMaxAt(const std::shared_ptr<State> &state, number t);

        /**
         * @brief Point-wise pruning.
         * 
         */
        void prune(number t = 0);

        bool is_dominated(const std::shared_ptr<State> &ostate, double value, number t);
    };
}
#include <sdm/utils/value_function/sawtooth_vf.tpp>
