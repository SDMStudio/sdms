
#pragma once

#include <sdm/utils/value_function/backup/backup_base.hpp>

namespace sdm
{
    class MaxPlanBackup : public BackupBase<std::shared_ptr<State>>
    {
    public:
        using TData = double;

        MaxPlanBackup();
        MaxPlanBackup(std::shared_ptr<SolvableByHSVI> );

        TData getBackup(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t);
        std::pair<double, std::shared_ptr<State>> getMaxAt(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t);
        std::shared_ptr<Action> getBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t);

    };
}