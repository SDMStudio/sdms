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
#include <boost/bimap.hpp>

#include <sdm/types.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/state/history.hpp>
#include <sdm/core/state/occupancy_state.hpp>

namespace sdm
{
  /**
   * @brief A private state of occupancy refers to a 
   * 
   * @tparam TState refers to a number
   * @tparam TJointHistory_p refers to a joint histories
   */
  template <typename TState = number, typename TJointHistory_p = JointHistoryTree_p<number>>
  class PrivateOccupancyState : public OccupancyState<TState, TJointHistory_p>
  {
  public:
    using jhistory_type = typename OccupancyState<TState, TJointHistory_p>::jhistory_type;
    using ihistory_type = typename jhistory_type::element_type::ihistory_type;
    using state_type = typename OccupancyState<TState, TJointHistory_p>::state_type;

    PrivateOccupancyState();
    PrivateOccupancyState(double);
    PrivateOccupancyState(number, double);
    PrivateOccupancyState(const PrivateOccupancyState &);
    PrivateOccupancyState(const OccupancyState<TState, TJointHistory_p> &);

    number getAgentId() const;
    // std::string str() const;
    const std::vector<ihistory_type> &getPartialJointHistory(const TJointHistory_p &) const;
    TJointHistory_p getJointHistory(const std::vector<ihistory_type> &) const;

    void finalize(bool = true);

    bool operator==(const PrivateOccupancyState &) const;

  protected:
    std::vector<ihistory_type> getPartialJointHistory(const std::vector<ihistory_type> &) const;

    /**
     * @brief The agent's identifier 
     */
    number agent_id_;

    typedef boost::bimaps::bimap<TJointHistory_p, Joint<ihistory_type>> bimap_type;
    typedef typename bimap_type::value_type bimap_value;

    /**
     * @brief Bimap that map joint histories and hash of o^{-i} 
     */
    bimap_type bimap_jhist_partial_jhist;
  };
} // namespace sdm
#include <sdm/core/state/private_occupancy_state.tpp>
