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
  class BeliefState : public MappedVector<number, double>
  {
  public:
    using state_type = number;

    BeliefState();
    BeliefState(double default_value);
    BeliefState(std::size_t size, double default_value);
    BeliefState(const BeliefState &v);

    static number getState(const number &state);
  };
} // namespace sdm

namespace std
{
    template <>
    struct hash<sdm::BeliefState>
    {
        typedef sdm::BeliefState argument_type;
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