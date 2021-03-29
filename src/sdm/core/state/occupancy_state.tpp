#include <sdm/core/state/occupancy_state.hpp>

namespace sdm
{

    template <typename TState, typename TJointHistory_p>
    OccupancyState<TState, TJointHistory_p>::OccupancyState() : MappedVector<Pair<TState, TJointHistory_p>, double>(0, 0)
    {
    }

    template <typename TState, typename TJointHistory_p>
    OccupancyState<TState, TJointHistory_p>::OccupancyState(double default_value) : MappedVector<Pair<TState, TJointHistory_p>, double>(default_value)
    {
    }

    template <typename TState, typename TJointHistory_p>
    OccupancyState<TState, TJointHistory_p>::OccupancyState(std::size_t size, double default_value) : MappedVector<Pair<TState, TJointHistory_p>, double>(size, default_value)
    {
    }

    template <typename TState, typename TJointHistory_p>
    OccupancyState<TState, TJointHistory_p>::OccupancyState(const OccupancyState &v) : MappedVector<Pair<TState, TJointHistory_p>, double>(v)
    {
    }

    template <typename TState, typename TJointHistory_p>
    std::set<typename OccupancyState<TState, TJointHistory_p>::jhistory_type> OccupancyState<TState, TJointHistory_p>::getJointHistories() const
    {
        std::set<jhistory_type> possible_jhistories;
        for (auto &key : *this)
        {
            possible_jhistories.insert(key.first.second);
        }
        return possible_jhistories;
    }

    template <typename TState, typename TJointHistory_p>
    std::set<typename OccupancyState<TState, TJointHistory_p>::state_type> OccupancyState<TState, TJointHistory_p>::getStates() const
    {
        std::set<state_type> possible_states;
        for (auto &key : *this)
        {
            possible_states.insert(key.first.first);
        }
        return possible_states;
    }

    template <typename TState, typename TJointHistory_p>
    std::vector<std::set<typename OccupancyState<TState, TJointHistory_p>::jhistory_type::element_type::ihistory_type>> OccupancyState<TState, TJointHistory_p>::getIndividualHistories() const
    {
        std::vector<std::set<typename OccupancyState<TState, TJointHistory_p>::jhistory_type::element_type::ihistory_type>> possible_ihistories;
        bool first_passage = true;
        for (auto &jhist : this->getJointHistories())
        {
            auto ihists = jhist->getIndividualHistories();
            for (std::size_t i = 0; i < ihists.size(); i++)
            {
                if (first_passage)
                {
                    possible_ihistories.push_back({});
                }

                possible_ihistories[i].insert(ihists[i]);
            }
            first_passage = false;
        }
        return possible_ihistories;
    }
} // namespace sdm