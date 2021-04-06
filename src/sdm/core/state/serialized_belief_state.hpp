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
  };

} // namespace sdm

// namespace std
// {
//     template <>
//     struct hash<sdm::BeliefState>
//     {
//         typedef sdm::BeliefState argument_type;
//         typedef std::size_t result_type;
//         inline result_type operator()(const argument_type &in) const
//         {
//             size_t seed = 0;
//             for (auto &v : in)
//             {
//                 //Combine the hash of the current vector with the hashes of the previous ones
//                 sdm::hash_combine(seed, v);
//             }
//             return seed;
//         }
//     };
// }