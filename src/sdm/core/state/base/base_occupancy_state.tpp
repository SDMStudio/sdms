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
        : MappedVector<Pair<TState, TJointHistory_p>, double>(v)
    {
        this->list_states = v.list_states;
        this->list_jhistories = v.list_jhistories;
        this->list_jhistory_states = v.list_jhistory_states;
        this->all_list_ihistories = v.all_list_ihistories;
        this->num_agents = v.num_agents;       
    }

    template <typename TState, typename TJointHistory_p>
    void BaseOccupancyState<TState, TJointHistory_p>::setProbabilityAt(const Pair<TState, TJointHistory_p> &pair_state_hist, double proba)
    {
        // Set the new occupancy measure
        (*this)[pair_state_hist] = proba;
    }

    template <typename TState, typename TJointHistory_p>
    void BaseOccupancyState<TState, TJointHistory_p>::setProbabilityAt(const TState &state, const TJointHistory_p &jhist, double proba)
    {
        this->setProbabilityAt(Pair<TState, TJointHistory_p>(state, jhist), proba);
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
    void BaseOccupancyState<TState, TJointHistory_p>::addProbabilityAt(const TState &state, const TJointHistory_p &jhist, double proba)
    {
        this->addProbabilityAt(Pair<TState, TJointHistory_p>(state, jhist), proba);
    }

    template <typename TState, typename TJointHistory_p>
    const std::vector<std::set<typename BaseOccupancyState<TState, TJointHistory_p>::jhistory_type::element_type::ihistory_type>> &BaseOccupancyState<TState, TJointHistory_p>::getAllIndividualHistories() const
    {
        return this->all_list_ihistories;
    }

    template <typename TState, typename TJointHistory_p>
    void BaseOccupancyState<TState, TJointHistory_p>::setAllIndividualHistories()
    {
        this->all_list_ihistories.clear();
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
    void BaseOccupancyState<TState, TJointHistory_p>::setJointHistories()
    {
        // Get the set of joint histories that are in the support of the BaseOccupancyState
        this->list_jhistories.clear();
        for (const auto &key : *this)
        {
            this->list_jhistories.insert(key.first.second);
        }
    }

    template <typename TState, typename TJointHistory_p>
    const std::set<TState> &BaseOccupancyState<TState, TJointHistory_p>::getStatesAt(const TJointHistory_p &jhistory) const
    {
        return this->list_jhistory_states.at(jhistory);
    }

    template <typename TState, typename TJointHistory_p>
    const std::set<typename BaseOccupancyState<TState, TJointHistory_p>::state_type> &BaseOccupancyState<TState, TJointHistory_p>::getStates() const
    {
        return this->list_states;
    }

    template <typename TState, typename TJointHistory_p>
    void BaseOccupancyState<TState, TJointHistory_p>::setStates()
    {
        for (const auto &pair_s_o : *this)
        {
            this->list_states.insert(pair_s_o.first.first);
            this->list_jhistory_states[pair_s_o.first.second].insert(pair_s_o.first.first);
        }
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
    double BaseOccupancyState<TState, TJointHistory_p>::getProbability(const Pair<TState, TJointHistory_p> &pair_state_hist) const
    {
        // Set the new occupancy measure
        return this->at(pair_state_hist);
    }

    template <typename TState, typename TJointHistory_p>
    void BaseOccupancyState<TState, TJointHistory_p>::finalize()
    {
        this->setStates();
        this->setJointHistories();
        this->setAllIndividualHistories();
    }

    template <typename TState, typename TJointHistory_p>
    std::shared_ptr<BaseOccupancyState<TState, TJointHistory_p>> BaseOccupancyState<TState, TJointHistory_p>::getptr()
    {
        return this->shared_from_this();
    }

    template <typename TState, typename TJointHistory_p>
    std::string BaseOccupancyState<TState, TJointHistory_p>::str() const
    {

        std::ostringstream res, tmp;
        number horizon;
        std::unordered_map<TJointHistory_p, std::pair<double, MappedVector<TState, double>>> map;
        for (const auto &pair_x_o_p : *this)
        {
            if (map.find(pair_x_o_p.first.second) == map.end())
            {
                map.emplace(pair_x_o_p.first.second, std::make_pair(0, MappedVector<TState, double>()));
                horizon = pair_x_o_p.first.second->getDepth();
            }
            map[pair_x_o_p.first.second].first += pair_x_o_p.second;
            map[pair_x_o_p.first.second].second[pair_x_o_p.first.first] = pair_x_o_p.second;
        }

        for (const auto &pair_x_o_p : *this)
        {
            map[pair_x_o_p.first.second].second[pair_x_o_p.first.first] /= map[pair_x_o_p.first.second].first;
        }


        res << "<occupancy-state size=\"" << map.size() << "\" horizon=\"" << horizon << "\">" << std::endl;
        for (const auto pair_o_pair_proba_belief : map)
        {
            auto joint_hist = pair_o_pair_proba_belief.first;
            res << "\t<joint-history value=\"" << joint_hist->short_str() << "\" proba=" << pair_o_pair_proba_belief.second.first << " belief=" << pair_o_pair_proba_belief.second.second << "/>" << std::endl;
        }
        res << "</occupancy-state>" << std::endl;
 
        return res.str();
    }

    template <typename TState, typename TJointHistory_p>
    std::string BaseOccupancyState<TState, TJointHistory_p>::str_hyperplan() const
    {

        std::ostringstream res, tmp;
        number horizon;

        std::unordered_set<TJointHistory_p> set;
        for (const auto &pair_x_o_p : *this)
        {
            set.emplace(pair_x_o_p.first.second);
            horizon = pair_x_o_p.first.second->getDepth();
        }

        res << "<simulataneous-hyperplan size=\"" << set.size() << "\" horizon=\"" << horizon << "\">" << std::endl;
        for (const auto joint_hist : set)
        {
            res << "\t<joint-history name=\"" << joint_hist->short_str() << "\" vector ="<< MappedVector<Pair<TState, TJointHistory_p>, double>::str()<< "/>" << std::endl;
        }
        res << "</simulataneous-hyperplan>" << std::endl;
 
        return res.str();
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