/**
 * @file belief_state.cpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief 
 * @version 1.0
 * @date 29/03/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <algorithm>

#include <sdm/types.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>

namespace sdm
{
  template <typename TState>
  class BaseBeliefState : public MappedVector<TState, double>
  {
  public:
    using state_type = TState;
    using value_type = typename MappedVector<TState, double>::value_type;

    BaseBeliefState();
    BaseBeliefState(double);
    BaseBeliefState(std::size_t, double);
    BaseBeliefState(const BaseBeliefState &);
    BaseBeliefState(std::initializer_list<value_type>);

    void setProbabilityAt(const TState &, double);
    void addProbabilityAt(const TState &, double);

    static TState getState(const TState &);

    friend std::ostream &operator<<(std::ostream &os, BaseBeliefState &state)
    {
      os << "<belief " << state.str() << "/>";
      return os;
    }
  };

  using BeliefState = BaseBeliefState<number>;
} // namespace sdm

#include <sdm/core/state/belief_state.tpp>