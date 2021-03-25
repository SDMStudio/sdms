#include <sdm/core/state/serialized_occupancy_state.hpp>

namespace sdm
{

    template <typename TState, typename TJointHistory_p>
    SerializedOccupancyState<TState, TJointHistory_p>::SerializedOccupancyState() : MappedVector<Tuple<TState, TJointHistory_p, std::vector<number>>, double>(0, 0)
    {
    }

    template <typename TState, typename TJointHistory_p>
    SerializedOccupancyState<TState, TJointHistory_p>::SerializedOccupancyState(double default_value) : MappedVector<Tuple<TState, TJointHistory_p, std::vector<number>>, double>(default_value)
    {
    }

    template <typename TState, typename TJointHistory_p>
    SerializedOccupancyState<TState, TJointHistory_p>::SerializedOccupancyState(std::size_t size, double default_value) : MappedVector<Tuple<TState, TJointHistory_p, std::vector<number>>, double>(size, default_value)
    {
    }

    template <typename TState, typename TJointHistory_p>
    SerializedOccupancyState<TState, TJointHistory_p>::SerializedOccupancyState(const SerializedOccupancyState &v) : MappedVector<Tuple<TState, TJointHistory_p, std::vector<number>>, double>(v)
    {
    }

    template <typename TState, typename TJointHistory_p>
    std::set<typename SerializedOccupancyState<TState, TJointHistory_p>::jhistory_type> SerializedOccupancyState<TState, TJointHistory_p>::getJointHistories() const
    {
        std::set<jhistory_type> possible_jhistories;
        for (const auto &key : *this)
        {
            possible_jhistories.insert(std::get<1>(key.first));
        }
        return possible_jhistories;
    }

    template <typename TState, typename TJointHistory_p>
    std::set<typename SerializedOccupancyState<TState, TJointHistory_p>::state_type> SerializedOccupancyState<TState, TJointHistory_p>::getStates() const
    {
        std::set<state_type> possible_states;
        for (const auto &key : *this)
        {
            possible_states.insert(std::get<0>(key.first));
        }
        return possible_states;
    }

    template <typename TState, typename TJointHistory_p>
    std::set<typename SerializedOccupancyState<TState, TJointHistory_p>::jhistory_type::element_type::ihistory_type> SerializedOccupancyState<TState, TJointHistory_p>::getIndividualHistories(number ag_id) const
    {
        std::set<typename SerializedOccupancyState<TState, TJointHistory_p>::jhistory_type::element_type::ihistory_type> possible_ihistories;
        for (const auto &key : *this)
        {
            auto jhist = std::get<1>(key.first);
            auto ihist = jhist->getIndividualHistory(ag_id);
            possible_ihistories.insert(ihist);
        }
        return possible_ihistories;
    }

    template <typename TState, typename TJointHistory_p>
    number SerializedOccupancyState<TState, TJointHistory_p>::getCurrentAgentId() const
    {
        return std::get<2>(this->begin()->first).size();
    }

} // namespace sdm