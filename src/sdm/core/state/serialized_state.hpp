#pragma once

#include <string>

#include <sdm/types.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/state/state.hpp>

namespace sdm
{
  template <typename TState>
  class SerializedState : public MappedVector<Pair<TState, std::vector<number>>, double>
  {
  public:
    //using jhistory_type = TJointHistory_p;
    using state_type = TState;

    SerializedState();
    SerializedState(double default_value);
    SerializedState(std::size_t size, double default_value);
    SerializedState(const SerializedState &v);

    //std::set<jhistory_type> getJointHistories() const;
    std::set<state_type> getStates() const;

    //std::set<typename jhistory_type::element_type::ihistory_type> getIndividualHistories(number ag_id) const;

    number getCurrentAgentId() const;
  };
} // namespace sdm
#include <sdm/core/state/serialized_state.tpp>


namespace std
{
    template <typename S>
    struct hash<sdm::SerializedState<S>>
    {
        typedef sdm::SerializedState<S> argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type &in) const
        {
            size_t seed = 0;
            for (auto &v : in)
            {
                //Combine the hash of the current vector with the hashes of the previous ones
                sdm::hash_combine(seed, v.first);
                sdm::hash_combine(seed, v.second);
            }
            return seed;
        }
    };
}