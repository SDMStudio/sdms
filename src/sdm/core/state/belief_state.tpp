#include <sdm/core/state/belief_state.hpp>

namespace sdm
{
  template <typename TState>
  BaseBeliefState<TState>::BaseBeliefState()
  {
  }

  template <typename TState>
  BaseBeliefState<TState>::BaseBeliefState(double default_value) : MappedVector<TState, double>(default_value)
  {
  }

  template <typename TState>
  BaseBeliefState<TState>::BaseBeliefState(std::initializer_list<typename MappedVector<TState, double>::value_type> list_values) : MappedVector<TState, double>(list_values)
  {
  }

  template <typename TState>
  BaseBeliefState<TState>::BaseBeliefState(std::size_t size, double default_value) : MappedVector<TState, double>(size, default_value)
  {
  }

  template <typename TState>
  BaseBeliefState<TState>::BaseBeliefState(const BaseBeliefState &v) : MappedVector<TState, double>(v)
  {
  }

  template <typename TState>
  void BaseBeliefState<TState>::setProbabilityAt(const TState &state, double proba)
  {
    // Set the new occupancy measure
    (*this)[state] = proba;
  }

  template <typename TState>
  void BaseBeliefState<TState>::addProbabilityAt(const TState &state, double proba)
  {
    if (this->find(state) != this->end())
    {
      (*this)[state] += proba;
    }
    else
    {
      this->setProbabilityAt(state, proba);
    }
  }

  template <typename TState>
  TState BaseBeliefState<TState>::getState(const TState &state)
  {
    return state;
  }

} // namespace sdm

namespace std
{

  template <typename TState>
  struct hash<sdm::BaseBeliefState<TState>>
  {
    typedef sdm::BaseBeliefState<TState> argument_type;
    typedef std::size_t result_type;
    inline result_type operator()(const argument_type &in) const
    {
      return std::hash<sdm::MappedVector<TState, double>>()(in);
    }
  };
}