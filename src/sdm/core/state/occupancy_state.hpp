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

#include <sdm/types.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/utils/struct/recursive_map.hpp>
#include <sdm/core/joint.hpp>
#include <sdm/core/state/history.hpp>
#include <sdm/core/state/base/base_occupancy_state.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>

namespace sdm
{

  /**
   * @brief An occupancy state refers to the whole knowledge that a central planner can have access to take decisions.
   * 
   * @tparam TState refers to a number
   * @tparam TJointHistory_p refers to a joint histories
   */
  template <typename TState = number, typename TJointHistory_p = JointHistoryTree_p<number>>
  class OccupancyState
      : public BaseOccupancyState<TState, TJointHistory_p>
  {
  public:
    using jhistory_type = typename BaseOccupancyState<TState, TJointHistory_p>::jhistory_type;
    using ihistory_type = typename jhistory_type::element_type::ihistory_type;
    using state_type = typename BaseOccupancyState<TState, TJointHistory_p>::state_type;
    using private_ostate_type = PrivateOccupancyState<TState, jhistory_type>;

    OccupancyState();
    OccupancyState(double);
    OccupancyState(std::size_t, double);
    OccupancyState(const OccupancyState &);

    auto compress();
    bool areIndividualHistoryLPE(const typename TJointHistory_p::element_type::ihistory_type &, const typename TJointHistory_p::element_type::ihistory_type &, number);
    void finalize();

    const Joint<RecursiveMap<ihistory_type, std::shared_ptr<PrivateOccupancyState<TState, TJointHistory_p>>>> &getPrivateOccupancyStates() const;
    const std::shared_ptr<PrivateOccupancyState<TState, TJointHistory_p>> &getPrivateOccupancyState(const number &agent_id, const ihistory_type &ihistory) const;

    /**
     * @brief Get the fully uncompressed occupancy state
     */
    std::shared_ptr<OccupancyState> getFullyUncompressedOccupancy() const;
    void setFullyUncompressedOccupancy(const std::shared_ptr<OccupancyState> &);

    std::shared_ptr<OccupancyState> getOneStepUncompressedOccupancy() const;
    void setOneStepUncompressedOccupancy(const std::shared_ptr<OccupancyState> &);

    std::vector<ihistory_type> getJointLabels(const std::vector<ihistory_type> &) const;

    std::shared_ptr<OccupancyState<TState, TJointHistory_p>> getptr();

  protected:
    /** @brief This representation of occupancy states consists of private occupancy states for each agent*/
    Joint<RecursiveMap<ihistory_type, std::shared_ptr<PrivateOccupancyState<TState, TJointHistory_p>>>> tuple_of_maps_from_histories_to_private_occupancy_states_;

    /** @brief Keep in memory the uncompressed occupancy states */
    std::shared_ptr<OccupancyState> fully_uncompressed_occupancy_state, one_step_left_compressed_occupancy_state;

    /** @brief Keep relations between all private ihistories and labels */
    Joint<RecursiveMap<ihistory_type, ihistory_type>> private_ihistory_map_;
  };
} // namespace sdm
#include <sdm/core/state/occupancy_state.tpp>