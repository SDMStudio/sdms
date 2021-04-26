#include <sdm/core/state/base/base_occupancy_state.hpp>

namespace sdm
{

    template <typename TState, typename TJointHistory_p>
    BaseOccupancyState<TState, TJointHistory_p>::BaseOccupancyState() : MappedVector<Pair<TState, TJointHistory_p>, double>(0, 0)
    {
    }

    template <typename TState, typename TJointHistory_p>
    BaseOccupancyState<TState, TJointHistory_p>::BaseOccupancyState(double default_value) : MappedVector<Pair<TState, TJointHistory_p>, double>(default_value)
    {
    }

    template <typename TState, typename TJointHistory_p>
    BaseOccupancyState<TState, TJointHistory_p>::BaseOccupancyState(std::size_t size, double default_value) : MappedVector<Pair<TState, TJointHistory_p>, double>(size, default_value)
    {
    }

    template <typename TState, typename TJointHistory_p>
    BaseOccupancyState<TState, TJointHistory_p>::BaseOccupancyState(const BaseOccupancyState &v) : MappedVector<Pair<TState, TJointHistory_p>, double>(v)
    {
    }

    template <typename TState, typename TJointHistory_p>
    void BaseOccupancyState<TState, TJointHistory_p>::setJointHistories()
    {
        // Get the set of joint histories that are in the support of the BaseOccupancyState
        if(this->joint_history_space.empty())
        {
            for (const auto &key : *this)
            {
                this->joint_history_space.insert(key.first.second);
            }
        }
    }

    template <typename TState, typename TJointHistory_p>
    std::set<typename BaseOccupancyState<TState, TJointHistory_p>::jhistory_type> BaseOccupancyState<TState, TJointHistory_p>::getJointHistories() const
    {
        return this->joint_history_space;
    }

    template <typename TState, typename TJointHistory_p>
    void BaseOccupancyState<TState, TJointHistory_p>::setStates()
    {
        // Get the set of states that are in the support of the BaseOccupancyState
        if(this->state_space.empty())
        {
            for (const auto &key : *this)
            {
                this->state_space.insert(key.first.first);
            }
        }
    }

    template <typename TState, typename TJointHistory_p>
    std::set<typename BaseOccupancyState<TState, TJointHistory_p>::state_type> BaseOccupancyState<TState, TJointHistory_p>::getStates() const
    {
        //assert( (!this->state_space.empty()) );
        return this->state_space;
    }

    template <typename TState, typename TJointHistory_p>
    void BaseOccupancyState<TState, TJointHistory_p>::setAllIndividualHistories()
    {
       if(this->agent_history_spaces.empty())
        {
            bool first_passage = true;
            for (const auto &jhist : this->getJointHistories()) // for all joint history in the support
            {
                auto ihists = jhist->getIndividualHistories(); // get associated individual histories for each agent
                for (std::size_t i = 0; i < ihists.size(); i++)
                {
                    // Init the set for agent i
                    if (first_passage)
                    {
                        this->agent_history_spaces.push_back({});
                    }
                    // Add the indiv history of agent i in his set
                    this->agent_history_spaces[i].insert(ihists[i]);
                }
                first_passage = false;
            }
        }
    }

    template <typename TState, typename TJointHistory_p>
    std::vector<std::set<typename BaseOccupancyState<TState, TJointHistory_p>::jhistory_type::element_type::ihistory_type>> BaseOccupancyState<TState, TJointHistory_p>::getAllIndividualHistories() const
    {
        //assert( (!this->agent_history_spaces.empty()) );
        return this->agent_history_spaces;
    }

    template <typename TState, typename TJointHistory_p>
    std::set<typename BaseOccupancyState<TState, TJointHistory_p>::jhistory_type::element_type::ihistory_type> BaseOccupancyState<TState, TJointHistory_p>::getIndividualHistories(number ag_id) const
    {
        //assert( (!this->agent_history_spaces[ag_id].empty()) );
        return this->agent_history_spaces[ag_id];
    }

    template <typename TState, typename TJointHistory_p>
    TState BaseOccupancyState<TState, TJointHistory_p>::getState(const Pair<TState, TJointHistory_p> &pair_state_hist) const
    {
        return pair_state_hist.first;
    }

    template <typename TState, typename TJointHistory_p>
    TState BaseOccupancyState<TState, TJointHistory_p>::getHiddenState(const Pair<TState, TJointHistory_p> &pair_state_hist) const
    {
        return this->getState(pair_state_hist);
    }



    template <typename TState, typename TJointHistory_p>
    TJointHistory_p BaseOccupancyState<TState, TJointHistory_p>::getHistory(const Pair<TState, TJointHistory_p> &pair_state_hist) const
    {
        return pair_state_hist.second;
    }

    template <typename TState, typename TJointHistory_p>
    double BaseOccupancyState<TState, TJointHistory_p>::getProbability(const Pair<TState, TJointHistory_p> &index)
    {
        return this->at(index);
    }

    template <typename TState, typename TJointHistory_p>
    void BaseOccupancyState<TState, TJointHistory_p>::setProbability(const Pair<TState, TJointHistory_p> & pair_state_jhistory, double probability)
    {
        (*this)[pair_state_jhistory] = probability;
    }

    template <typename TState, typename TJointHistory_p>
    void BaseOccupancyState<TState, TJointHistory_p>::finalizing()
    {
        this->setStates();
        this->setJointHistories();
        this->setAllIndividualHistories();
    }

} // namespace sdm

namespace std
{
    template <typename S, typename V>
    struct hash<sdm::BaseOccupancyState<S, V>>
    {
        typedef sdm::BaseOccupancyState<S, V> argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type &in) const
        {
            return std::hash<sdm::MappedVector<sdm::Pair<S, V>, double>>()(in);
        }
    };
}