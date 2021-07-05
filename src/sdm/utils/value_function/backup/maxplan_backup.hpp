
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

        virtual TData backup(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state,const std::shared_ptr<Action>& action, number t);
    protected : 

        TData setHyperplanBelief(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t);
        TData setHyperplanOccupancy(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t);

    };
}