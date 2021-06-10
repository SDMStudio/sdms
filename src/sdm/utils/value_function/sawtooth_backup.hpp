
#pragma once

#include <sdm/utils/value_function/backup_base.hpp>
#include <sdm/utils/value_function/value_function.hpp>

namespace sdm
{
    class SawtoothBackup : public BackupBase<double>
    {
    public:
        using TData = double;

        SawtoothBackup();
        SawtoothBackup(std::shared_ptr<SolvableByHSVI> );

        TData getBackup(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t);
        std::pair<double, std::shared_ptr<State>> getMaxAt(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t);
        std::shared_ptr<Action> getBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t);

    };
}
#include <sdm/utils/value_function/sawtooth_backup.tpp>
