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

#include <sdm/types.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>

namespace sdm
{
  template <typename TState>
  class BaseBeliefState : public MappedVector<TState, double>
  {
  public:
    using state_type = number;

    BaseBeliefState();
    BaseBeliefState(double default_value);
    BaseBeliefState(std::size_t size, double default_value);
    BaseBeliefState(const BaseBeliefState &v);

    static TState getState(const TState &state);
  };

  using BeliefState = BaseBeliefState<number>;
} // namespace sdm

#include <sdm/core/state/belief_state.tpp>

namespace std
{
    template <typename TState>
    struct hash<sdm::BaseBeliefState<TState>>
    {
        typedef sdm::BaseBeliefState<TState> argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type &in) const
        {
            size_t seed = 0;
            for (auto &v : in)
            {
                //Combine the hash of the current vector with the hashes of the previous ones
                sdm::hash_combine(seed, v);
            }
            return seed;
        }
    };
}