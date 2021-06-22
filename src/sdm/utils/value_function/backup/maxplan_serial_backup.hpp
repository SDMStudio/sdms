#pragma once

#include <sdm/utils/value_function/backup/maxplan_backup.hpp>

namespace sdm
{
    class MaxPlanSerialBackup : public MaxPlanBackup
    {
    public:
        using TData = std::shared_ptr<State>;

        MaxPlanSerialBackup();
        MaxPlanSerialBackup(const std::shared_ptr<SolvableByHSVI>& );

        virtual TData backup(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t);
        virtual std::shared_ptr<Action> getBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t);

        /**
         * @brief return the value for a precise decision rule and next hyperplan
         * 
         * @param const std::shared_ptr<SerialOccupancyInterface> & :serial_occupancy_state 
         * @param const std::shared_ptr<DecisionRule>& : action 
         * @param const std::shared_ptr<SerialOccupancyInterface> & :next_step_hyperplan 
         * @param number : t 
         * @return double 
         */
        double getMaxPlanValueAt(const std::shared_ptr<SerialOccupancyInterface> &serial_occupancy_state, const std::shared_ptr<Action>& action, const std::shared_ptr<SerialOccupancyInterface>& next_step_hyperplan, number t);
    protected : 
        Pair<std::shared_ptr<State>,std::shared_ptr<Action>> getBestActionAndMaxHyperplan(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t);

    };
}