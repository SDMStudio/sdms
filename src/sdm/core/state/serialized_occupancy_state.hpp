#pragma once

#include <sdm/types.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/state/serialized_state.hpp>
#include <sdm/core/state/history.hpp>
#include <sdm/core/state/serialized_belief_state.hpp>

namespace sdm
{
  /**
   * @brief A serialized occupancy state refers to an occupancy state (i.e refers to the whole knowledge that a central planner can have access to take decisions) with a precise state for each agent. 
   * But in this implementation we call serialized occupancy state a distribution over serialized state and joint histories .
   * 
   * @tparam TState
   * @tparam TJointHistory_p 
   */
  template <typename TState = SerializedState, typename TJointHistory_p = JointHistoryTree_p<number>>
  class SerializedOccupancyState : public OccupancyState<TState, TJointHistory_p>
  {
  protected:
    number agent;

  public:
    using state_type = typename OccupancyState<TState, TJointHistory_p>::state_type;
    using jhistory_type = typename OccupancyState<TState, TJointHistory_p>::jhistory_type;

    SerializedOccupancyState(double default_value);
    SerializedOccupancyState(number num_agents = 2, double default_value = 0.);
    SerializedOccupancyState(const SerializedOccupancyState &);
    SerializedOccupancyState(const OccupancyState<TState, TJointHistory_p> &);

    std::string str() const;
    std::string str_hyperplan() const;

    /**
     * @brief Return the agent id of the serial occupancy state
     * 
     * @return number 
     */
    number getCurrentAgentId() const;
    /**
     * @brief Set the Agent if of the serial occupancy state
     * 
     * @param number : agent id
     */
    void setAgent(number);

    /**
     * @brief Get the Hidden state of a precise element of the occupancy state
     * 
     * @param const Pair<state_type, jhistory_type> & : element of occupancy state 
     * @return state_type::state_type 
     */
    typename state_type::state_type getHiddenState(const Pair<state_type, jhistory_type> &) const;

    /**
     * @brief Get the Action object of a precise element of the occupancy state
     * 
     * @param const Pair<state_type, jhistory_type> & ! elmement of occupancy state 
     * @return std::vector<typename state_type::action_type> 
     */
    std::vector<typename state_type::action_type> getAction(const Pair<state_type, jhistory_type> &state) const;

    std::shared_ptr<SerializedOccupancyState<TState, TJointHistory_p>> getptr();

    std::shared_ptr<SerializedOccupancyState> getOneStepUncompressedOccupancy() const;
    std::shared_ptr<SerializedOccupancyState> getFullyUncompressedOccupancy() const;

    /**
     * @brief Create of a belief conditionning to a joint history
     * 
     * @return MappedVector<TState,double> 
     */
    const SerializedBeliefState createBelief(const TJointHistory_p& ) const;

    /**
     * @brief Create a belief weighted conditionning to a joint history
     * 
     * @return MappedVector<TState,double> 
     */
    const SerializedBeliefState createBeliefWeighted(const TJointHistory_p&) const;
  };
} // namespace sdm
#include <sdm/core/state/serialized_occupancy_state.tpp>