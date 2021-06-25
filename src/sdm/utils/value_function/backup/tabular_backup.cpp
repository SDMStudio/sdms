#include <sdm/utils/value_function/backup/tabular_backup.hpp>

namespace sdm
{
    TabularBackup::TabularBackup(){}

    TabularBackup::TabularBackup(const std::shared_ptr<SolvableByHSVI>& world) : BackupBase<double>(world) {}

    double TabularBackup::backup(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, const std::shared_ptr<Action>& action, number t)
    {
        return this->world_->getReward(state, action, t) + this->world_->getDiscount(t) * this->world_->getExpectedNextValue(vf->getptr(), state, action, t);
    }
}