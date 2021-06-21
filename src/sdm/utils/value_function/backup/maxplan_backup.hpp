
#pragma once

#include <sdm/utils/value_function/backup/backup_base.hpp>

namespace sdm
{
    class MaxPlanBackup : public BackupBase<std::shared_ptr<State>>
    {
    public:
        using TData = std::shared_ptr<State>;

        MaxPlanBackup();
        MaxPlanBackup(const std::shared_ptr<SolvableByHSVI>& );

        virtual TData backup(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t);
        virtual std::pair<double, std::shared_ptr<State>> getMaxAt(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t);
        virtual std::shared_ptr<Action> getBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t);


        // For the moment, both were write in different function, but in the futur we will regroup them
        TData backupBeliefState(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t);
        TData backupOccupancyState(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t);

        TData getHyperplanAt(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State> &state, const std::shared_ptr<BeliefInterface> &next_hyperplan, const std::shared_ptr<Action> &action, number t);

    };
}