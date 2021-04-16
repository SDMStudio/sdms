#include <sdm/core/state/base_occupancy_state.hpp>

namespace sdm
{

    template <typename TState, typename THistory_p>
    BaseOccupancyState<TState, THistory_p>::BaseOccupancyState() : MappedVector<Pair<TState, THistory_p>, double>(0, 0)
    {
    }

    template <typename TState, typename THistory_p>
    BaseOccupancyState<TState, THistory_p>::BaseOccupancyState(double default_value) : MappedVector<Pair<TState, THistory_p>, double>(
        default_value
    )
    {
    }

    template <typename TState, typename THistory_p>
    BaseOccupancyState<TState, THistory_p>::BaseOccupancyState(
        std::size_t size, double default_value
    ) : MappedVector<Pair<TState, THistory_p>, double>(size, default_value)
    {
    }

    template <typename TState, typename THistory_p>
    BaseOccupancyState<TState, THistory_p>::BaseOccupancyState(const BaseOccupancyState &v) : MappedVector<Pair<TState, THistory_p>, double>(v)
    {
    }

    template <typename TState, typename THistory_p>
    std::set<typename BaseOccupancyState<TState, THistory_p>::history_type> BaseOccupancyState<TState, THistory_p>::getHistories(
    ) const
    {
        std::set<history_type> possible_histories;
        for (const auto &key : *this)
        {
            possible_histories.insert(key.first.second);
        }
        return possible_histories;
    }

    template <typename TState, typename THistory_p>
    std::set<typename BaseOccupancyState<TState, THistory_p>::state_type> BaseOccupancyState<TState, THistory_p>::getStates() const
    {
        std::set<state_type> possible_states;
        for (const auto &key : *this)
        {
            possible_states.insert(key.first.first);
        }
        return possible_states;
    }

    template <typename TState, typename THistory_p>
    TState BaseOccupancyState<TState, THistory_p>::getState(const Pair<TState, THistory_p> &pair_state_hist) const
    {
        return pair_state_hist.first;
    }

    template <typename TState, typename THistory_p>
    TState BaseOccupancyState<TState, THistory_p>::getHiddenState(const Pair<TState, THistory_p> &pair_state_hist) const
    {
        return this->getState(pair_state_hist);
    }

    template <typename TState, typename THistory_p>
    THistory_p BaseOccupancyState<TState, THistory_p>::getHistory(const Pair<TState, THistory_p> &pair_state_hist) const
    {
        return pair_state_hist.second;
    }

    template <typename TState, typename THistory_p>
    double BaseOccupancyState<TState, THistory_p>::getProbability(const Pair<TState, THistory_p> &)
    {
        for (const auto &key : *this)
        {
            //possible_states.insert(key.first.first);
        }        
        return 0.0;
    }

} // namespace sdm