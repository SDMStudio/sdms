namespace sdm
{
    template <typename TState>
    BaseState<TState>::BaseState() {}

    template <typename TState>
    BaseState<TState>::BaseState(const TState &item) : state(item) {}

    template <typename TState>
    BaseState<TState>::~BaseState() {}

    template <typename TState>
    TState BaseState<TState>::getState() const
    {
        return this->state;
    }

    template <typename TState>
    void BaseState<TState>::setState(const TState &state)
    {
        this->state = state;
    }

    template <typename TState>
    std::string BaseState<TState>::str() const
    {
        std::ostringstream res;
        res << "State(" << this->state << ")";
        return res.str();
    }

    template <typename TState>
    bool BaseState<TState>::operator==(const BaseState &other) const
    {
        return (this->getState() == other.getState());
    }

    template <typename TState>
    TypeState BaseState<TState>::getTypeState() const 
    {
      return TypeState::STATE;
    }


} // namespace sdm


namespace std
{

  template <typename TState>
  struct hash<sdm::BaseState<TState>>
  {
    typedef sdm::BaseState<TState> argument_type;
    typedef std::size_t result_type;
    inline result_type operator()(const argument_type &in) const
    {
      return std::hash<TState>()(in.getState());
    }
  };
}