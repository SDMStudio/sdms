#include <sdm/config.hpp>

namespace sdm
{
  template <typename TState, template <typename TI, typename TV> class TVector>
  double BaseBeliefState<TState, TVector>::PRECISION = config::PRECISION_BELIEF;

  template <typename TState, template <typename TI, typename TV> class TVector>
  BaseBeliefState<TState, TVector>::BaseBeliefState()
  {
  }

  template <typename TState, template <typename TI, typename TV> class TVector>
  BaseBeliefState<TState, TVector>::BaseBeliefState(double default_value) : TVector<TState, double>(default_value)
  {
  }

  template <typename TState, template <typename TI, typename TV> class TVector>
  BaseBeliefState<TState, TVector>::BaseBeliefState(std::size_t size, double default_value) : TVector<TState, double>(size, default_value)
  {
  }

  template <typename TState, template <typename TI, typename TV> class TVector>
  BaseBeliefState<TState, TVector>::BaseBeliefState(std::initializer_list<BaseBeliefState<TState, TVector>::value_type> list_values) : TVector<TState, double>(list_values)
  {
  }

  template <typename TState, template <typename TI, typename TV> class TVector>
  BaseBeliefState<TState, TVector>::BaseBeliefState(const std::vector<TState> &list_states, const std::vector<double> &list_proba) : BaseBeliefState(list_states.size())
  {
    assert(list_states.size() == list_proba.size());
    for (size_t i = 0; i < list_states.size(); i++)
    {
      this->setProbabilityAt(list_states[i], list_proba[i]);
    }
  }

  template <typename TState, template <typename TI, typename TV> class TVector>
  BaseBeliefState<TState, TVector>::BaseBeliefState(const BaseBeliefState &v) : TVector<TState, double>(v)
  {
  }

  template <typename TState, template <typename TI, typename TV> class TVector>
  BaseBeliefState<TState, TVector>::BaseBeliefState(const TVector<TState, double> &v) : TVector<TState, double>(v)
  {
  }

  template <typename TState, template <typename TI, typename TV> class TVector>
  void BaseBeliefState<TState, TVector>::setProbabilityAt(const TState &state, double proba)
  {
    // Set the new occupancy measure
    this->setValueAt(state, proba);
  }

  template <typename TState, template <typename TI, typename TV> class TVector>
  double BaseBeliefState<TState, TVector>::getProbabilityAt(const TState &state) const
  {
    return this->getValueAt(state);
  }

  template <typename TState, template <typename TI, typename TV> class TVector>
  void BaseBeliefState<TState, TVector>::addProbabilityAt(const TState &state, double proba)
  {
    this->setValueAt(state, this->getProbabilityAt(state) + proba);
  }

  template <typename TState, template <typename TI, typename TV> class TVector>
  TState BaseBeliefState<TState, TVector>::getState(const TState &state)
  {
    return state;
  }

  template <typename TState, template <typename TI, typename TV> class TVector>
  bool BaseBeliefState<TState, TVector>::operator==(const BaseBeliefState &other) const
  {
    return TVector<TState, double>::is_equal(other, PRECISION);
  }

  template <typename TState, template <typename TI, typename TV> class TVector>
  std::string BaseBeliefState<TState, TVector>::str() const
  {
    {
      std::ostringstream res;
      res << "[" << this->size() << "](";
      int i = 0;
      for (const auto &pair_state_proba : *this)
      {
        res << ((i == 0) ? "" : ", ");
        res << pair_state_proba.first << " --> " << pair_state_proba.second;
        i++;
      }
      res << ")";
      return res.str();
    }
  }

  template <typename TState, template <typename TI, typename TV> class TVector>
  template <class Archive>
  void BaseBeliefState<TState, TVector>::serialize(Archive &archive, const unsigned int)
  {
    archive &boost::serialization::base_object<TVector<TState, double>>(*this);
  }

} // namespace sdm

namespace std
{

  template <typename TState, template <typename TI, typename TV> class TVector>
  struct hash<sdm::BaseBeliefState<TState, TVector>>
  {
    typedef sdm::BaseBeliefState<TState, TVector> argument_type;
    typedef std::size_t result_type;
    inline result_type operator()(const argument_type &in) const
    {
      return std::hash<TVector<TState, double>>()(in);
    }
  };
}