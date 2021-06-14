#pragma once

#include <sdm/utils/value_function/backup/backup_base.hpp>

namespace sdm
{
    class TabularBackup : public BackupBase<double>
    {
    public:
        using TData = double;

        TabularBackup();
        TabularBackup(std::shared_ptr<SolvableByHSVI> world);

        virtual ~TabularBackup();

        std::pair<double, std::shared_ptr<State>> getMaxAt(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t);
        TData backup(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t);
        std::shared_ptr<Action> getBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t);
    };
}

