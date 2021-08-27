#pragma once

#include <sdm/utils/value_function/backup/backup_base.hpp>

namespace sdm
{
    class TabularBackup : public BackupBase<double>
    {
    public:
        using TData = double;

        TabularBackup();
        TabularBackup(const std::shared_ptr<SolvableByHSVI>& world);

        TData backup(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, const std::shared_ptr<Action>& action, number t);
    };
}

