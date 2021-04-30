#include <sdm/core/state/private_occupancy_state.hpp>

namespace sdm
{

    template <typename TState, typename TJointJointHistory_p>
    PrivateOccupancyState<TState, 
                          TJointJointHistory_p>::PrivateOccupancyState() : MappedVector<Pair<TState, 
                                                                                        TJointJointHistory_p>, double>(0, 0)
    {
    }

    template <typename TState, typename TJointJointHistory_p>
    PrivateOccupancyState<TState, TJointJointHistory_p>::PrivateOccupancyState(
        double default_value) : MappedVector<Pair<TState, TJointJointHistory_p>, double>(default_value)
    {
    }

    template <typename TState, typename TJointJointHistory_p>
    PrivateOccupancyState<TState, TJointJointHistory_p>::PrivateOccupancyState(
        std::size_t size, double default_value) : MappedVector<Pair<TState, TJointJointHistory_p>, double>(size, default_value)
    {
    }

    template <typename TState, typename TJointJointHistory_p>
    PrivateOccupancyState<TState, TJointJointHistory_p>::PrivateOccupancyState(
        const PrivateOccupancyState &v) : MappedVector<Pair<TState, TJointJointHistory_p>, double>(v)
    {
    }

    template <typename TState, typename TJointJointHistory_p>
    std::set<typename PrivateOccupancyState<TState, 
        TJointJointHistory_p>::jjhistory_type> PrivateOccupancyState<TState, TJointJointHistory_p>::getJointJointHistories() const
    {
        std::set<jjhistory_type> possible_jjhistories;
        for (const auto &key : *this)
        {
            possible_jjhistories.insert(key.first.second);
        }
        return possible_jjhistories;
    }

    template <typename TState, typename TJointJointHistory_p>
    std::set<typename PrivateOccupancyState<TState, 
        TJointJointHistory_p>::state_type> PrivateOccupancyState<TState, TJointJointHistory_p>::getStates() const
    {
        std::set<state_type> possible_states;
        for (const auto &key : *this)
        {
            possible_states.insert(key.first.first);
        }
        return possible_states;
    }

    template <typename TState, typename TJointJointHistory_p>
    std::vector<
        std::set<
            typename PrivateOccupancyState<
                TState, TJointJointHistory_p>::jhistory_type>> PrivateOccupancyState<
                    TState, TJointJointHistory_p>::getAllIndividualJointHistories() const
    {
        std::vector<std::set<typename PrivateOccupancyState<TState, 
            TJointJointHistory_p>::jhistory_type>> possible_jhistories;
        bool first_passage = true;
        for (const auto &jjhist : this->getJointJointHistories())
        {
            auto jhists = jjhist;
            for (std::size_t i = 0; i < jhists.size(); i++)
            {
                if (first_passage)
                {
                    possible_jhistories.push_back({});
                }

                possible_jhistories[i].insert(jhists[i]);
            }
            first_passage = false;
        }
        return possible_jhistories;
    }

    template <typename TState, typename TJointJointHistory_p>
    std::set<typename PrivateOccupancyState<TState, 
        TJointJointHistory_p>::jhistory_type> PrivateOccupancyState<TState, 
            TJointJointHistory_p>::getIndividualJointHistories(number ag_id) const
    {
        std::vector<std::set<jhistory_type>> All_jhistories = getAllIndividualJointHistories();
        return All_jhistories[ag_id];
    }

    template <typename TState, typename TJointJointHistory_p>
    TState PrivateOccupancyState<TState, TJointJointHistory_p>::getState(const Pair<TState, TJointJointHistory_p> &pair_state_hist) const
    {
        return pair_state_hist.first;
    }

    template <typename TState, typename TJointJointHistory_p>
    TState PrivateOccupancyState<TState, TJointJointHistory_p>::getHiddenState(const Pair<TState, TJointJointHistory_p> &pair_state_hist) const
    {
        return this->getState(pair_state_hist);
    }

    template <typename TState, typename TJointJointHistory_p>
    TJointJointHistory_p PrivateOccupancyState<TState, TJointJointHistory_p>::getHistory(const Pair<TState, TJointJointHistory_p> &pair_state_hist) const
    {
        return pair_state_hist.second;
    }

    template <typename TState, typename TJointJointHistory_p>
    double PrivateOccupancyState<TState, TJointJointHistory_p>::getProbability(const Pair<TState, TJointJointHistory_p> &)
    {
        for (const auto &key : *this)
        {
            //possible_states.insert(key.first.first);
        }        
        return 0.0;
    }

} // namespace sdm