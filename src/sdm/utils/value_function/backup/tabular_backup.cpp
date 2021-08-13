#include <sdm/utils/value_function/backup/tabular_backup.hpp>
#include <sdm/world/belief_mdp.hpp>

namespace sdm
{
    TabularBackup::TabularBackup() {}

    TabularBackup::TabularBackup(const std::shared_ptr<SolvableByHSVI>& world) : BackupBase<double>(world) {}

    double TabularBackup::backup(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, const std::shared_ptr<Action>& action, number t)
    {
        return this->world_->getReward(state, action, t) + this->world_->getDiscount(t) * this->world_->getExpectedNextValue(vf, state, action, t);
    }

    double TabularBackup::backup_relaxed(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, const std::shared_ptr<Action>& action, number t)
    {
        return this->world_->getReward(state, action, t) + this->world_->getDiscount(t) * std::static_pointer_cast<BeliefMDP>(this->world_)->getExpectedNextValueRelaxed(vf, state, action, t);
    }
}