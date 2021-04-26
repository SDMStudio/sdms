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

namespace sdm
{
  template <typename TState>
  class BaseBeliefState : public MappedVector<TState, double>
  {
  public:
    using state_type = number;

    BaseBeliefState();
    BaseBeliefState(double);
    BaseBeliefState(std::size_t, double );
    BaseBeliefState(const BaseBeliefState &);

    static TState getState(const TState &);
  };

  using BeliefState = BaseBeliefState<number>;
} // namespace sdm

#include <sdm/core/state/belief_state.tpp>