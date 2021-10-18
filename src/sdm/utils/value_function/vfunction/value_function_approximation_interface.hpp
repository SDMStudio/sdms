#pragma once

#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/utils/value_function/update_operator/vupdate_operator.hpp>

namespace sdm
{
    class ValueFunctionApproximationInterface : public ValueFunction
    {
    public:
        ValueFunctionApproximationInterface(number horizon = 0, const std::shared_ptr<Initializer> &initializer = nullptr,
                                            const std::shared_ptr<ActionSelectionInterface> &action = nullptr,
                                            const std::shared_ptr<UpdateOperatorInterface> &update_operator = nullptr,
                                            int freq_pruning = -1);

        /**
         * @brief Apply the pruning process to the whole structure.
         *
         * In algorithms, this method should be invoked at each trial. If the current
         * trial fit with the pruning frequency, then we update the value function and
         * prune unecessary components.
         *
         * @param trial the current trial
         */
        void doPruning(number trial);

        /**
         * @brief Get the pruning frequency.
         * 
         * @return the pruning frequency
         */
        int getPruningFrequency() const;

    protected:
        /**
         * @brief Prune unecessary components of the value function.
         *
         * This function will prune the dominated hyperplanes.
         *
         * @param t the time step
         */
        virtual void prune(number t) = 0;

        /**
         * @brief Frequency of pruning.
         */
        int freq_pruning;
    };
} // namespace sdm