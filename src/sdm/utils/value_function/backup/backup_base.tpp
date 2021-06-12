#include <sdm/utils/value_function/backup/backup_base.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>

namespace sdm
{
    template <typename TData>
    BackupBase<TData>::BackupBase() {}

    template <typename TData>
    BackupBase<TData>::BackupBase(std::shared_ptr<SolvableByHSVI> world) : world_(world) {}

    template <typename TData>
    BackupBase<TData>::~BackupBase() {}

    template <typename TData>
    std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> BackupBase<TData>::getQValueAt(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t)
    {
        // Compute Q(s,*)
        std::shared_ptr<MappedVector<std::shared_ptr<Action>, double>> qvalue = std::make_shared<MappedVector<std::shared_ptr<Action>, double>>();
        for (const auto &action : *world_->getActionSpaceAt(state, t))
        {
            auto casted_action = action->toAction();
            (*qvalue)[casted_action] = this->getQValueAt(vf, state, casted_action, t);
        }
        return qvalue;
    }

    template <typename TData>
    double BackupBase<TData>::getQValueAt(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        return world_->getReward(state, action, t) + world_->getDiscount(t) * world_->getExpectedNextValue(vf->getptr(), state, action, t);
    }
}