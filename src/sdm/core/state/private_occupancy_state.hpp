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
   * @brief A private state of occupancy refers to a 
   * 
   * @tparam TState refers to a number
   * @tparam TJointHistory_p refers to a joint histories
   */
  template <typename TState = number, typename TJointHistory_p = HistoryTree_p<number>>
  class PrivateOccupancyState : public MappedVector<Pair<TState, TJointHistory_p>, double>
  {
  public:
    using jhistory_type = TJointHistory_p;
    using state_type = TState;

    PrivateOccupancyState();
    PrivateOccupancyState(double default_value);
    PrivateOccupancyState(std::size_t size, double default_value);
    PrivateOccupancyState(const PrivateOccupancyState &v);

    std::set<state_type> getStates() const;

    std::set<jhistory_type> getHistories() const;
    std::vector<std::set<typename jhistory_type::element_type::ihistory_type>> getAllIndividualHistories() const;
    std::set<typename jhistory_type::element_type::ihistory_type> getIndividualHistories(number ag_id) const;

    /**
     * @brief Return the hidden state of a precise occupancy state
     * 
     * @param pair_state_hist refers to a precise occupancy state
     * @return TState refers to the hidden state returned
     */
    TState getState(const Pair<TState, TJointHistory_p> &pair_state_hist) const;

    /**
     * @brief Get the Hidden State of a precise occupancy state. For this situation, the hidden state is the current State
     * 
     * @param pair_state_hist 
     * @return TState 
     */
    TState getHiddenState(const Pair<TState, TJointHistory_p> &pair_state_hist) const;

    /**
     * @brief Return the hidden  history of a precise occupancy state
     * 
     * @param pair_state_hist refers to a precise occupancy state
     * @return TJointHistory_p refers to the hidden  history returned
     */
    TJointHistory_p getHistory(const Pair<TState, TJointHistory_p> &pair_state_hist) const;

    /**
     * @brief Return the probability of a precise occupancy state
     * 
     * @param pair_state_hist refers to a precise occupancy state
     * @return double refers to the probability returned
     */
    double getProbability(const Pair<TState, TJointHistory_p> &pair_state_hist);

  };
} // namespace sdm
#include <sdm/core/state/private_occupancy_state.tpp>

namespace std
{
    template <typename S, typename V>
    struct hash<sdm::PrivateOccupancyState<S, V>>
    {
        typedef sdm::PrivateOccupancyState<S, V> argument_type;
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