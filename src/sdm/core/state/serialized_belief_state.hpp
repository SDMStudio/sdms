#pragma once

#include <sdm/types.hpp>
#include <sdm/core/state/belief_state.hpp>
#include <sdm/core/state/serialized_state.hpp>

namespace sdm
{
  class SerializedBeliefState : public BaseBeliefState<SerializedState>
  {
  public:
    using state_type = SerializedState;
    using action_type = number;

    SerializedBeliefState();
    SerializedBeliefState(double default_value);
    SerializedBeliefState(std::size_t size, double default_value);
    SerializedBeliefState(const SerializedBeliefState &v);

    number getCurrentAgentId() const;
    void setAgent(number);

    /**
     * @brief Get the hidden State of a precise SerializedState
     * 
     * @param state refers to a precise SerializedState
     * @return state_type::state_type refers to the hidden state
     */
    typename state_type::state_type getHiddenState(const state_type &state) const;

    /**
     * @brief Get the hidden vector of action of a precise SerializedState
     * 
     * @param state refers to a precise SerializedState
     * @return std::vector<typename state_type::action_type> refers to the hidden vector of action
     */
    std::vector<typename state_type::action_type> getAction(const state_type &state) const;

    protected : 

      number agent;

  };

} // namespace sdm