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

namespace sdm
{
  template <typename TState, typename TJointHistory_p>
  class PrivateOccupancyState;

  /**
   * @brief An occupancy state refers to the whole knowledge that a central planner can have access to take decisions.
   * 
   * @tparam TState refers to a number
   * @tparam TJointHistory_p refers to a joint histories
   */
  template <typename TState = number, typename TJointHistory_p = JointHistoryTree_p<number>>
  class OccupancyState : public BaseOccupancyState<TState, TJointHistory_p>
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

    /**
     * @brief Get the private occupancy state structure. This structure consists of a set of private occupancy state for each agent. 
     */
    const Joint<RecursiveMap<ihistory_type, std::shared_ptr<PrivateOccupancyState<TState, TJointHistory_p>>>> &getPrivateOccupancyStates() const;

    /**
     * @brief Get a specific private occupancy state. The private occupancy state return is for a specific agent and a specific history of this agent. 
     * 
     * @param agent_id the agent id
     * @param ihistory the individual history
     * @return the corresponding private occupancy state
     */
    const std::shared_ptr<PrivateOccupancyState<TState, TJointHistory_p>> &getPrivateOccupancyState(const number &agent_id, const ihistory_type &ihistory) const;

    /**
     * @brief Get the fully uncompressed occupancy state.
     */
    std::shared_ptr<OccupancyState> getFullyUncompressedOccupancy() const;

    /**
     * @brief Set the fully uncompressed occupancy state.
     */
    void setFullyUncompressedOccupancy(const std::shared_ptr<OccupancyState> &);

    /**
     * @brief Get the one step uncompressed occupancy state. 
     */
    std::shared_ptr<OccupancyState> getOneStepUncompressedOccupancy() const;

    /**
     * @brief Set the one step uncompressed occupancy state
     */
    void setOneStepUncompressedOccupancy(const std::shared_ptr<OccupancyState> &);

    /**
     * @brief Get the label that corresponds to the ihistory.
     */
    ihistory_type getLabel(const ihistory_type &ihistory, number agent_id) const;

    /**
     * @brief Get the list of labels that corresponds to the list of ihistories.
     */
    std::vector<ihistory_type> getJointLabels(const std::vector<ihistory_type> &) const;

    /**
     * @brief Update the label of a specific individual history
     */
    void updateLabel(number, const ihistory_type &, const ihistory_type &);

    /**
     * @brief Update the labels of multiple individual histories
     */
    void updateJointLabels(const std::vector<ihistory_type> &, const std::vector<ihistory_type> &);

    /**
     * @brief Get the Compressed Joint History. 
     */
    TJointHistory_p getCompressedJointHistory(const TJointHistory_p &) const;

    std::shared_ptr<OccupancyState<TState, TJointHistory_p>> getptr();

    /**
     * @brief Get the probability over individual histories and precise agent
     * 
     * @param number Agent Id
     * @param typename jhistory_type::element_type::ihistory_type : Individual History
     */
    const double &getProbabilityOverIndividualHistories(number,typename jhistory_type::element_type::ihistory_type) const;
    void setProbabilityOverIndividualHistories();


  protected:
    /** @brief This representation of occupancy states consists of private occupancy states for each agent*/
    Joint<RecursiveMap<ihistory_type, std::shared_ptr<PrivateOccupancyState<TState, TJointHistory_p>>>> tuple_of_maps_from_histories_to_private_occupancy_states_;

    /** @brief Keep in memory the uncompressed occupancy states */
    std::shared_ptr<OccupancyState> fully_uncompressed_occupancy_state, one_step_left_compressed_occupancy_state;

    /** @brief Keep relations between all private ihistories and labels */
    Joint<RecursiveMap<ihistory_type, std::shared_ptr<ihistory_type>>> private_ihistory_map_;

    /** @brief */
    Joint<RecursiveMap<ihistory_type, std::shared_ptr<ihistory_type>>> map_label_to_pointer;

    /** @brief Keep relation between list of individual histories and joint histories */
    RecursiveMap<std::vector<ihistory_type>, jhistory_type> jhistory_map_;

    /** @brief probability of a private history space for a precise agent */
    std::unordered_map<number, std::unordered_map<ihistory_type, double>> probability_ihistories;

  };
} // namespace sdm
#include <sdm/core/state/occupancy_state.tpp>