#include <sdm/core/state/serialized_state.hpp>

namespace sdm
{

    template <typename TState>
    SerializedState<TState>::SerializedState() : MappedVector<Pair<TState, std::vector<number>>, double>(0, 0)
    {
    }

    template <typename TState>
    SerializedState<TState>::SerializedState(double default_value) : MappedVector<Pair<TState, std::vector<number>>, double>(default_value)
    {
    }

    template <typename TState>
    SerializedState<TState>::SerializedState(std::size_t size, double default_value) : MappedVector<Pair<TState, std::vector<number>>, double>(size, default_value)
    {
    }

    template <typename TState>
    SerializedState<TState>::SerializedState(const SerializedState &v) : MappedVector<Pair<TState, std::vector<number>>, double>(v)
    {
    }

    /*
    template <typename TState>
    std::set<typename SerializedState<TState>::jhistory_type> SerializedState<TState>::getJointHistories() const
    {
        std::set<jhistory_type> possible_jhistories;
        for (const auto &key : *this)
        {
            possible_jhistories.insert(sdm::get<1>(key.first));
        }
        return possible_jhistories;
    }*/

    template <typename TState>
    std::set<typename SerializedState<TState>::state_type> SerializedState<TState>::getStates() const
    {
        std::set<state_type> possible_states;
        for (const auto &key : *this)
        {
            possible_states.insert(sdm::get<0>(key.first));
        }
        return possible_states;
    }

    /*

    template <typename TState>
    std::set<typename SerializedState<TState, TJoitHistory_p>::jhistory_type::element_type::ihistory_type> SerializedState<TState, TJointHistory_p>::getIndividualHistories(number ag_id) const
    {
        std::set<typename SerializedState<TState, TJointHistory_p>::jhistory_type::element_type::ihistory_type> possible_ihistories;
        for (const auto &key : *this)
        {
            auto jhist = sdm::get<1>(key.first);
            auto ihist = jhist->getIndividualHistory(ag_id);
            possible_ihistories.insert(ihist);
        }
        return possible_ihistories;
    }*/

    template <typename TState>
    number SerializedState<TState>::getCurrentAgentId() const
    {
        return (this->begin()->first).second.size();
    }

} // namespace sdm