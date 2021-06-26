namespace sdm
{
    template <typename TData>
    BackupBase<TData>::BackupBase() {}

    template <typename TData>
    BackupBase<TData>::BackupBase(const std::shared_ptr<SolvableByHSVI> &world) : world_(world) {}

    template <typename TData>
    BackupBase<TData>::~BackupBase() {}

    template <typename TData>
    std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> BackupBase<TData>::getQValueAt(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t)
    {
        // Compute Q(s,*)
        std::shared_ptr<MappedVector<std::shared_ptr<Action>, double>> q_s = std::make_shared<MappedVector<std::shared_ptr<Action>, double>>();
        auto action_space = this->world_->getActionSpaceAt(state, t);
        for (const auto &action : *action_space)
        {
            auto casted_action = std::static_pointer_cast<Action>(action);
            (*q_s)[casted_action] = this->getQValueAt(vf, state, casted_action, t);
        }
        return q_s;
    }

    template <typename TData>
    double BackupBase<TData>::getQValueAt(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        double value = world_->getReward(state, action, t) + world_->getDiscount(t) * world_->getExpectedNextValue(vf->getptr(), state, action, t);
        return value;
    }
}