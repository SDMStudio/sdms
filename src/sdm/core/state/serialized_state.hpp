#pragma once

#include <sdm/types.hpp>
#include <sdm/utils/struct/vector.hpp>
#include <sdm/utils/struct/pair.hpp>

namespace sdm
{
  template <typename TState = number, typename TAction = number>
  class SerializedState : public Pair<TState, std::vector<TAction>>
  {
  public:
    using state_type = TState;
    using action_type = TAction;

    SerializedState();
    SerializedState(TState state, std::vector<TAction> actions);
    SerializedState(const SerializedState &v);

    TState getState() const;
    std::vector<TAction> getAction() const;
    number getCurrentAgentId() const;
  };
} // namespace sdm

#include <sdm/core/state/serialized_state.tpp>

namespace std
{
  template <typename S, typename A>
  struct hash<sdm::SerializedState<S, A>>
  {
    typedef sdm::SerializedState<S, A> argument_type;
    typedef std::size_t result_type;
    inline result_type operator()(const argument_type &in) const
    {
      size_t seed = 0;
      //Combine the hash of the current vector with the hashes of the previous ones
      sdm::hash_combine(seed, in.first);
      sdm::hash_combine(seed, in.second);
      return seed;
    }
  };
}