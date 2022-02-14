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
  size_t BaseState<TData>::hash(double precision) const
  {
    return std::hash<TData>()(this->state);
  }

  template <typename TData>
  bool BaseState<TData>::isEqual(const BaseState &other, double) const
  {
    return (this->getState() == other.getState());
  }

  template <typename TData>
  bool BaseState<TData>::isEqual(const std::shared_ptr<State> &other, double precision) const
  {
    auto other_oalpha = std::static_pointer_cast<BaseState<TData>>(other);
    if (other_oalpha == nullptr)
      return false;
    else
      return this->isEqual(*other_oalpha, precision);
  }
} // namespace sdm

namespace std
{
  template <typename TData>
  struct hash<sdm::BaseState<TData>>
  {
    inline std::size_t operator()(const sdm::BaseState<TData> &in, double precision) const
    {
      return in.hash(precision);
    }

    inline std::size_t operator()(const sdm::BaseState<TData> &in) const
    {
      return in.hash(0.);
    }
  };
}