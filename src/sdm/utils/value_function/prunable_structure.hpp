#pragma once

#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/utils/value_function/update_operator/vupdate_operator.hpp>

namespace sdm
{
    class PrunableStructure
    {
    public:
        PrunableStructure(number horizon = 0,int freq_pruning = -1);

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

        /**
         * @brief The horizon.
         */
        number horizon;
    };
} // namespace sdm