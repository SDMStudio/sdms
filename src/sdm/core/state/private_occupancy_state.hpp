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

namespace sdm
{

  /**
   * @brief A private state of occupancy refers to a 
   * 
   * @tparam TState refers to a number
   * @tparam TJointHistory_p refers to a joint histories
   */
  template <typename TState = number, typename TJointHistory_p = JointHistoryTree_p<number>>
  class PrivateOccupancyState : public BaseOccupancyState<TState, TJointHistory_p>
  {
  public:
    using jhistory_type = typename BaseOccupancyState<TState, TJointHistory_p>::jhistory_type;
    using ihistory_type = typename jhistory_type::element_type::ihistory_type;
    using state_type = typename BaseOccupancyState<TState, TJointHistory_p>::state_type;

    PrivateOccupancyState();
    PrivateOccupancyState(double);
    PrivateOccupancyState(number, double);
    PrivateOccupancyState(std::size_t, double);
    PrivateOccupancyState(const PrivateOccupancyState &);

    number getAgentId() const;
    std::string str() const;
    const std::vector<ihistory_type> &getPartialJointHistory(const TJointHistory_p &) const;
    TJointHistory_p getJointHistory(const std::vector<ihistory_type> &) const;

    void finalize();
    bool operator==(const PrivateOccupancyState &) const;

  protected:
    std::vector<ihistory_type> getPartialJointHistory(const std::vector<ihistory_type> &) const;

    typedef boost::bimaps::bimap<TJointHistory_p, Joint<ihistory_type>> bimap_type;
    typedef typename bimap_type::value_type bimap_value;

    /**
     * @brief Bimap that map joint histories and hash of o^{-i} 
     */
    bimap_type bimap_jhist_partial_jhist;

    /**
     * @brief The agent's identifier 
     */
    number agent_id_;
  };
} // namespace sdm
#include <sdm/core/state/private_occupancy_state.tpp>
