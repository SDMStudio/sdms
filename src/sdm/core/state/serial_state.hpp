#pragma once

#include <sdm/core/action/action.hpp>
#include <sdm/core/state/base_state.hpp>
#include <sdm/core/state/interface/serial_interface.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/utils/struct/vector.hpp>

namespace sdm
{
  /**
   * @brief The serial state class: a statistics for serial reformulations.
   *
   * Serial states are statistics used to solve serial games (an alternative reformulation of
   * simultaneous problems to drop the time complexity). A serial state is generally composed
   * of the classic state plus the list of actions chosen by the previous agents.
   *
   */
  class SerialState : public BaseState<Pair<std::shared_ptr<State>, Joint<std::shared_ptr<Action>>>>,
                      public BaseSerialInterface
  {
  public:
    SerialState();
    SerialState(std::shared_ptr<State> state, Joint<std::shared_ptr<Action>> actions);
    SerialState(const SerialState &v);
    virtual ~SerialState();

    /**
     * @brief Get the hidden state of the serial state.
     *
     * The hidden state can be seen as the classic state. This is the serial state minus
     * the list of actions made by previous agents.
     *
     * @return the hidden state
     */
    std::shared_ptr<State> getHiddenState() const;

    /**
     * @brief Get the list of actions that were already decided.
     *
     * This function will return the list of actions selected by previous agents in
     * the serial reformulation. This is the serial state minus the classic state.
     *
     * @return the list of (previous) actions
     */
    Joint<std::shared_ptr<Action>> getAction() const;

    /**
     * @brief Get the agent ID corresponding to the current state.
     *
     * @return the current agent ID
     */
    number getCurrentAgentId() const;

    /**
     * @brief Set the agent ID corresponding to the current state.
     *
     */
    void setAgentId(number agent_id);

    std::string str() const;

  protected:
    /**
     * @brief The identifier of the agent that owns this state.
     */
    number agentID_;
  };

} // namespace sdm

// namespace boost
// {
//   namespace serialization
//   {
//     template <class Archive>
//     void serialize(Archive &archive, sdm::SerialState &serial_state, const unsigned int)
//     {
//       // archive &boost::serialization::base_object<sdm::Pair<sdm::number, std::vector<sdm::number>>>(serial_state);
//     }

//   } // namespace serialization
// } // namespace boost

namespace std
{
  template <>
  struct hash<sdm::SerialState>
  {
    typedef sdm::SerialState argument_type;
    typedef std::size_t result_type;
    inline result_type operator()(const argument_type &in) const
    {
      size_t seed = 0;
      // Combine the hash of the current vector with the hashes of the previous ones
      sdm::hash_combine(seed, in.getHiddenState());
      sdm::hash_combine(seed, in.getAction());
      return seed;
    }
  };
}
