/**
 * @file occupancy_state.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief 
 * @version 1.0
 * @date 29/03/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <string>

#include <sdm/types.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/state/history.hpp>

namespace sdm
{

  /**
   * @brief A state of occupancy refers to a state in which all the knowledge is known by a central planner and which it 
   * relies on to make a decision.
   * 
   * @tparam TState refers to a number
   * @tparam THistory_p refers to a joint histories
   */
  template <typename TState, typename THistory_p>
  class BaseOccupancyState : public MappedVector<Pair<TState, THistory_p>, double>
  {
  public:
    using history_type = THistory_p;
    using state_type = TState;

    BaseOccupancyState();
    BaseOccupancyState(double default_value);
    BaseOccupancyState(std::size_t size, double default_value);
    BaseOccupancyState(const BaseOccupancyState &v);

    std::set<state_type> getStates() const;

    std::set<history_type> getHistories() const;

    /**
     * @brief Return the hidden state of a precise occupancy state
     * 
     * @param pair_state_hist refers to a precise occupancy state
     * @return TState refers to the hidden state returned
     */
    TState getState(const Pair<TState, THistory_p> &pair_state_hist) const;

    /**
     * @brief Get the Hidden State of a precise occupancy state. For this situation, the hidden state is the current State
     * 
     * @param pair_state_hist 
     * @return TState 
     */
    TState getHiddenState(const Pair<TState, THistory_p> &pair_state_hist) const;

    /**
     * @brief Return the hidden history of a precise occupancy state
     * 
     * @param pair_state_hist refers to a precise occupancy state
     * @return THistory_p refers to the hidden history returned
     */
    THistory_p getHistory(const Pair<TState, THistory_p> &pair_state_hist) const;

    /**
     * @brief Return the probability of a precise occupancy state
     * 
     * @param pair_state_hist refers to a precise occupancy state
     * @return double refers to the probability returned
     */
    double getProbability(const Pair<TState, THistory_p> &pair_state_hist);

  };
} // namespace sdm
#include <sdm/core/state/base_occupancy_state.tpp>

namespace std
{
    template <typename S, typename V>
    struct hash<sdm::BaseOccupancyState<S, V>>
    {
        typedef sdm::BaseOccupancyState<S, V> argument_type;
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