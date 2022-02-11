namespace sdm
{
    template <typename TData>
    BaseState<TData>::BaseState() {}

    template <typename TData>
    BaseState<TData>::BaseState(const TData &item) : state(item) {}

    template <typename TData>
    BaseState<TData>::~BaseState() {}

    template <typename TData>
    TData BaseState<TData>::getState() const
    {
        return this->state;
    }

    template <typename TData>
    void BaseState<TData>::setState(const TData &state)
    {
        this->state = state;
    }

    template <typename TData>
    std::string BaseState<TData>::str() const
    {
        std::ostringstream res;
        res << this->state;
        return res.str();
    }

    template <typename TData>
    bool BaseState<TData>::operator==(const BaseState &other) const
    {
        return (this->getState() == other.getState());
    }


} // namespace sdm


namespace std
{

  template <typename TData>
  struct hash<sdm::BaseState<TData>>
  {
    typedef sdm::BaseState<TData> argument_type;
    typedef std::size_t result_type;
    inline result_type operator()(const argument_type &in) const
    {
      return std::hash<TData>()(in.state);
    }
  };
}