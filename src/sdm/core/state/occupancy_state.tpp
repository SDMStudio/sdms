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
    bool OccupancyState<TState, TJointHistory_p>::areIndividualHistoryLPE(const typename TJointHistory_p::element_type::ihistory_type &hist1, const typename TJointHistory_p::element_type::ihistory_type &hist2, number ag_id)
    {
        /**
         * reference -> JAIR 2016 : https://www.jair.org/index.php/jair/article/view/10986/26136 (page 492-493)
         **/

        // Transform proba representation from s(x, \theta) to s(x, \theta^{-i} \mid \theta^i) and s(x, \theta^{-i} \mid \theta'^i)
        MappedVector<Pair<TState, Joint<typename TJointHistory_p::element_type::ihistory_type>>, double> proba_theta, proba_theta_;
        for (const auto &pair_s_o_p : *this)
        {
            // Transform representation of hist1 (= \theta^i)
            if (pair_s_o_p.first.second->getIndividualHistory(ag_id) == hist1)
            {
                // Get all individual histories except that of agent i
                auto other = pair_s_o_p.first.second->getIndividualHistories();
                other.erase(std::remove(other.begin(), other.end(), hist1), other.end());
                // Compute p(x, o^{-i} | o^i)
                proba_theta[std::make_pair(pair_s_o_p.first.first, other)] += pair_s_o_p.second;
            }

            // Transform representation of hist2 (= \theta'^i)
            if (pair_s_o_p.first.second->getIndividualHistory(ag_id) == hist2)
            {
                // Get all individual histories except that of agent i
                auto other = pair_s_o_p.first.second->getIndividualHistories();
                other.erase(std::remove(other.begin(), other.end(), hist2), other.end());
                // Compute p(x, o^{-i} | o'^i)
                proba_theta_[std::make_pair(pair_s_o_p.first.first, other)] += pair_s_o_p.second;
            }
        }

        // If p(x, o^{-i} | o^i) and p(x, o^{-i} | o'^i) are equal then histories are equivalent
        if (proba_theta != proba_theta_)
        {
            return false;
        }
        return true;
    }

    template <typename TState, typename TJointHistory_p>
    bool OccupancyState<TState, TJointHistory_p>::areStateJointHistoryPairsLPE(const Pair<TState, TJointHistory_p> &p1, const Pair<TState, TJointHistory_p> &p2)
    {
        /**
         * reference -> JAIR 2016 : https://www.jair.org/index.php/jair/article/view/10986/26136 (page 492-493)
         **/
        if (p1.first != p2.first)
        {
            return false;
        }
        for (number ag_id = 0; ag_id < p1.second->getIndividualHistories().size(); ag_id++)
        {
            if (!this->areIndividualHistoryLPE(p1.second->getIndividualHistory(ag_id), p2.second->getIndividualHistory(ag_id), ag_id))
            {
                return false;
            }
        }
        return true;
    }

    template <typename TState, typename TJointHistory_p>
    auto OccupancyState<TState, TJointHistory_p>::compress()
    {
        /**
         * reference -> JAIR 2016 : https://www.jair.org/index.php/jair/article/view/10986/26136 (page 492-493)
         **/
        OccupancyState<TState, TJointHistory_p> compact_ostate;
        auto support = this->getIndexes();
        // Sort support to get same label for cluster of equivalent histories but different states 
        std::sort(support.begin(), support.end());
        for (auto iter = support.begin(); iter != support.end();)
        {
            auto pair_s_o = *iter;
            compact_ostate[pair_s_o] = this->at(pair_s_o);
            iter = support.erase(iter);
            for (auto iter2 = iter; iter2 != support.end();)
            {
                auto pair_s_o_2 = *iter2;
                if (this->areStateJointHistoryPairsLPE(pair_s_o, pair_s_o_2))
                {
                    compact_ostate[pair_s_o] = compact_ostate[pair_s_o] + this->at(pair_s_o_2);
                    iter2 = support.erase(iter2);
                }
                else
                {
                    iter2++;
                }
            }
        }
        return compact_ostate;
    }

    template <typename TState, typename TJointHistory_p>
    std::set<typename OccupancyState<TState, TJointHistory_p>::jhistory_type> OccupancyState<TState, TJointHistory_p>::getJointHistories() const
    {
        // Get the set of joint histories that are in the support of the OccupancyState
        std::set<jhistory_type> possible_jhistories;
        for (const auto &key : *this)
        {
            possible_jhistories.insert(key.first.second);
        }
        return possible_jhistories;
    }

    template <typename TState, typename TJointHistory_p>
    std::set<typename OccupancyState<TState, TJointHistory_p>::state_type> OccupancyState<TState, TJointHistory_p>::getStates() const
    {
        // Get the set of states that are in the support of the OccupancyState
        std::set<state_type> possible_states;
        for (const auto &key : *this)
        {
            possible_states.insert(key.first.first);
        }
        return possible_states;
    }

    template <typename TState, typename TJointHistory_p>
    std::vector<std::set<typename OccupancyState<TState, TJointHistory_p>::jhistory_type::element_type::ihistory_type>> OccupancyState<TState, TJointHistory_p>::getAllIndividualHistories() const
    {
        std::vector<std::set<typename OccupancyState<TState, TJointHistory_p>::jhistory_type::element_type::ihistory_type>> possible_ihistories;

        bool first_passage = true;
        for (const auto &jhist : this->getJointHistories()) // for all joint history in the support
        {
            auto ihists = jhist->getIndividualHistories(); // get associated individual histories for each agent
            for (std::size_t i = 0; i < ihists.size(); i++)
            {
                // Init the set for agent i
                if (first_passage)
                {
                    possible_ihistories.push_back({});
                }
                // Add the indiv history of agent i in his set
                possible_ihistories[i].insert(ihists[i]);
            }
            first_passage = false;
        }
        return possible_ihistories;
    }

    template <typename TState, typename TJointHistory_p>
    std::set<typename OccupancyState<TState, TJointHistory_p>::jhistory_type::element_type::ihistory_type> OccupancyState<TState, TJointHistory_p>::getIndividualHistories(number ag_id) const
    {
        std::vector<std::set<typename jhistory_type::element_type::ihistory_type>> All_ihistories = getAllIndividualHistories();
        return All_ihistories[ag_id];
    }

    template <typename TState, typename TJointHistory_p>
    TState OccupancyState<TState, TJointHistory_p>::getState(const Pair<TState, TJointHistory_p> &pair_state_hist) const
    {
        return pair_state_hist.first;
    }

    template <typename TState, typename TJointHistory_p>
    TState OccupancyState<TState, TJointHistory_p>::getHiddenState(const Pair<TState, TJointHistory_p> &pair_state_hist) const
    {
        return this->getState(pair_state_hist);
    }

    template <typename TState, typename TJointHistory_p>
    TJointHistory_p OccupancyState<TState, TJointHistory_p>::getHistory(const Pair<TState, TJointHistory_p> &pair_state_hist) const
    {
        return pair_state_hist.second;
    }

    template <typename TState, typename TJointHistory_p>
    double OccupancyState<TState, TJointHistory_p>::getProbability(const Pair<TState, TJointHistory_p> &)
    {
        for (const auto &key : *this)
        {
            //possible_states.insert(key.first.first);
        }
        return 0.0;
    }

} // namespace sdm

namespace std
{
    template <typename S, typename V>
    struct hash<sdm::OccupancyState<S, V>>
    {
        typedef sdm::OccupancyState<S, V> argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type &in) const
        {
            return std::hash<sdm::MappedVector<sdm::Pair<S, V>, double>>()(in);
        }
    };
}