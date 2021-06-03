namespace sdm
{
    template <typename TState>
    BaseState<TState>::BaseState() {}

    template <typename TState>
    BaseState<TState>::BaseState(const TState &item) : state_(item) {}

    template <typename TState>
    BaseState<TState>::~BaseState() {}

    template <typename TState>
    TState BaseState<TState>::getState() const
    {
        return this->state_;
    }

    template <typename TState>
    void BaseState<TState>::setState(const TState &state)
    {
        this->state_ = state;
    }

    template <typename TState>
    std::string BaseState<TState>::str() const
    {
        std::ostringstream res;
        res << "State(" << this->state_ << ")";
        return res.str();
    }

} // namespace sdm
