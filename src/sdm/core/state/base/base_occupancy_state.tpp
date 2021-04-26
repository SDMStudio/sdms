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
    BaseOccupancyState<TState, TJointHistory_p>::BaseOccupancyState(const BaseOccupancyState &v)
        : MappedVector<Pair<TState, TJointHistory_p>, double>(v),
          list_states(v.getStates()),
          list_jhistories(v.getJointHistories()),
          all_list_ihistories(v.getAllIndividualHistories())
    {
    }

    template <typename TState, typename TJointHistory_p>
    void BaseOccupancyState<TState, TJointHistory_p>::setProbabilityAt(const Pair<TState, TJointHistory_p> &pair_state_hist, double proba)
    {
        // Set the new occupancy measure
        (*this)[pair_state_hist] = proba;
    }

    template <typename TState, typename TJointHistory_p>
    void BaseOccupancyState<TState, TJointHistory_p>::addProbabilityAt(const Pair<TState, TJointHistory_p> &pair_state_hist, double proba)
    {
        if (this->find(pair_state_hist) != this->end())
        {
            (*this)[pair_state_hist] += proba;
        }
        else
        {
            this->setProbabilityAt(pair_state_hist, proba);
        }
    }

    template <typename TState, typename TJointHistory_p>
    void BaseOccupancyState<TState, TJointHistory_p>::finalize()
    {
        // Set basic accessors 
        this->list_states.clear();
        this->list_jhistories.clear();
        this->all_list_ihistories.clear();

        for (const auto &key : *this)
        {
            this->list_states.insert(key.first.first);
            this->list_jhistories.insert(key.first.second);
        }

        bool first_passage = true;
        for (const auto &jhist : this->getJointHistories())
        {
            auto ihists = jhist->getIndividualHistories();
            for (std::size_t i = 0; i < ihists.size(); i++)
            {
                if (first_passage)
                {
                    this->all_list_ihistories.push_back({});
                }

                this->all_list_ihistories[i].insert(ihists[i]);
            }
            first_passage = false;
        }
    }

    template <typename TState, typename TJointHistory_p>
    const std::set<typename BaseOccupancyState<TState, TJointHistory_p>::jhistory_type> &BaseOccupancyState<TState, TJointHistory_p>::getJointHistories() const
    {
        // Get the set of joint histories that are in the support of the BaseOccupancyState
        return this->list_jhistories;
    }

    template <typename TState, typename TJointHistory_p>
    const std::set<typename BaseOccupancyState<TState, TJointHistory_p>::state_type> &BaseOccupancyState<TState, TJointHistory_p>::getStates() const
    {
        return this->list_states;
    }

    template <typename TState, typename TJointHistory_p>
    const std::vector<std::set<typename BaseOccupancyState<TState, TJointHistory_p>::jhistory_type::element_type::ihistory_type>> &BaseOccupancyState<TState, TJointHistory_p>::getAllIndividualHistories() const
    {
        return this->all_list_ihistories;
    }

    template <typename TState, typename TJointHistory_p>
    const std::set<typename BaseOccupancyState<TState, TJointHistory_p>::jhistory_type::element_type::ihistory_type> &BaseOccupancyState<TState, TJointHistory_p>::getIndividualHistories(number ag_id) const
    {
        return this->all_list_ihistories[ag_id];
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