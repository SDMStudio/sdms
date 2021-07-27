#pragma once

#include <sdm/types.hpp>

namespace sdm
{
    class State;
    class Action;

    class QValueBackupInterface
    {
    public:  

        /**
         * @brief 
         * 
         * @param number t : time step
         * @return 
         */
        virtual double update(number t) = 0;

        /**
         * @brief 
         * 
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return 
         */
        virtual std::shared_ptr<Action> getGreedyAction(const std::shared_ptr<State> &state, number t) = 0;

        /**
         * @brief 
         * 
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return 
         */
        virtual double getValueAt(const std::shared_ptr<State> &state, number t) = 0;
    };

} // namespace sdm
