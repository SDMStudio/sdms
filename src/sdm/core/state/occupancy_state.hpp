/**
 * @file hsvi.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief HSVI algorithm
 * @version 0.1
 * @date 22/12/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#pragma once

#include <string>

#include <sdm/types.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/state/state.hpp>

namespace sdm
{
  template <typename TState, typename TJointHistory_p>
  class OccupancyState : public MappedVector<Pair<TState, TJointHistory_p>, double>
  {
  public:
    using jhistory_type = TJointHistory_p;
    using state_type = TState;

    OccupancyState();
    OccupancyState(double default_value);
    OccupancyState(std::size_t size, double default_value);
    OccupancyState(const OccupancyState &v);

    std::set<jhistory_type> getJointHistories() const;

    std::set<state_type> getStates() const;

    std::vector<std::set<typename jhistory_type::element_type::ihistory_type>> getIndividualHistories() const;
  };
} // namespace sdm
#include <sdm/core/state/occupancy_state.tpp>


namespace std
{
    template <typename S, typename V>
    struct hash<sdm::OccupancyState<S, V>>
    {
        typedef sdm::OccupancyState<S, V> argument_type;
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