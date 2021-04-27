#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>

namespace sdm
{

    template <typename TState, typename TJointHistory_p>
    OccupancyState<TState, TJointHistory_p>::OccupancyState()
    {
    }

    template <typename TState, typename TJointHistory_p>
    OccupancyState<TState, TJointHistory_p>::OccupancyState(double default_value) : BaseOccupancyState<TState, TJointHistory_p>(default_value)
    {
    }

    template <typename TState, typename TJointHistory_p>
    OccupancyState<TState, TJointHistory_p>::OccupancyState(std::size_t size, double default_value) : BaseOccupancyState<TState, TJointHistory_p>(size, default_value)
    {
    }

    template <typename TState, typename TJointHistory_p>
    OccupancyState<TState, TJointHistory_p>::OccupancyState(const OccupancyState &occupancy_state) : BaseOccupancyState<TState, TJointHistory_p>(occupancy_state), private_omap_(occupancy_state.getPrivateOccupancyStates())
    {
    }

    template <typename TState, typename TJointHistory_p>
    const Joint<RecursiveMap<typename OccupancyState<TState, TJointHistory_p>::ihistory_type, PrivateOccupancyState<TState, TJointHistory_p>>>& OccupancyState<TState, TJointHistory_p>::getPrivateOccupancyStates() const
    {
        return this->private_omap_;
    }

    template <typename TState, typename TJointHistory_p>
    const PrivateOccupancyState<TState, TJointHistory_p>& OccupancyState<TState, TJointHistory_p>::getPrivateOccupancyState(const number &agent_id, const typename OccupancyState<TState, TJointHistory_p>::ihistory_type &ihistory) const
    {
        return this->private_omap_[agent_id][ihistory];
    }

    template <typename TState, typename TJointHistory_p>
    bool OccupancyState<TState, TJointHistory_p>::areIndividualHistoryLPE(const typename TJointHistory_p::element_type::ihistory_type &history_1, const typename TJointHistory_p::element_type::ihistory_type &history_2, number agent_identifier)
    {
        /**
         * reference -> JAIR 2016 : https://www.jair.org/index.php/jair/article/view/10986/26136 (page 492-493)
         **/
        // Transform proba representation from s(x, \theta) to s(x, \theta^{-i} \mid \theta^i) and s(x, \theta^{-i} \mid \theta'^i)
        MappedVector<Pair<TState, Joint<typename TJointHistory_p::element_type::ihistory_type>>, double> private_occupancy_state_1, private_occupancy_state_2;
        for (const auto &pair_state_history_probability : *this)
        {
            // Transform representation of history_1 (= \theta^i)
            if (pair_state_history_probability.first.second->getIndividualHistory(agent_identifier) == history_1)
            {
                // Get all individual histories except that of agent i
                auto agent_histories = pair_state_history_probability.first.second->getIndividualHistories();
                agent_histories.erase(std::remove(agent_histories.begin(), agent_histories.end(), history_1), agent_histories.end());
                // Compute p(x, o^{-i} | o^i)
                private_occupancy_state_1[std::make_pair(pair_state_history_probability.first.first, agent_histories)] += pair_state_history_probability.second;
            }

            // Transform representation of history_2 (= \theta'^i)
            if (pair_state_history_probability.first.second->getIndividualHistory(agent_identifier) == history_2)
            {
                // Get all individual histories except that of agent i
                auto agent_histories = pair_state_history_probability.first.second->getIndividualHistories();
                agent_histories.erase(std::remove(agent_histories.begin(), agent_histories.end(), history_2), agent_histories.end());
                // Compute p(x, o^{-i} | o'^i)
                private_occupancy_state_2[std::make_pair(pair_state_history_probability.first.first, agent_histories)] += pair_state_history_probability.second;
            }
        }

        // If p(x, o^{-i} | o^i) and p(x, o^{-i} | o'^i) are equal then histories are equivalent
        if (private_occupancy_state_1 != private_occupancy_state_2)
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
        for (number agent_identifier = 0; agent_identifier < p1.second->getIndividualHistories().size(); agent_identifier++)
        {
            if (!this->areIndividualHistoryLPE(p1.second->getIndividualHistory(agent_identifier), p2.second->getIndividualHistory(agent_identifier), agent_identifier))
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
            compact_ostate.setProbabilityAt(pair_s_o, this->at(pair_s_o));
            iter = support.erase(iter);
            for (auto iter2 = iter; iter2 != support.end();)
            {
                auto pair_s_o_2 = *iter2;
                if (this->areStateJointHistoryPairsLPE(pair_s_o, pair_s_o_2))
                {
                    compact_ostate.addProbabilityAt(pair_s_o, this->at(pair_s_o_2));
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
    void OccupancyState<TState, TJointHistory_p>::finalize()
    {
        BaseOccupancyState<TState, TJointHistory_p>::finalize();

        // Build the private occupancy map
        if (this->private_omap_.empty())
        {
            for (const auto &pair_state_jhist : *this)
            {
                auto state = this->getState(pair_state_jhist.first);
                auto jhist = this->getHistory(pair_state_jhist.first);
                auto proba = this->getProbability(pair_state_jhist.first);

                for (number ag = 0; ag < jhist->getIndividualHistories().size(); ag++)
                {
                    // Init map for agent i
                    if (this->private_omap_.size() <= ag)
                    {
                        this->private_omap_.push_back({});
                    }

                    // Get joint history except private one
                    auto other_histories = jhist->getIndividualHistories();
                    other_histories.erase(std::remove(other_histories.begin(), other_histories.end(), jhist->getIndividualHistory(ag)), other_histories.end());

                    // Set private OccupancyState
                    if (this->private_omap_[ag].find(jhist->getIndividualHistory(ag)) == this->private_omap_[ag].end())
                    {
                        this->private_omap_[ag].emplace(jhist->getIndividualHistory(ag), PrivateOccupancyState(this->default_value_));
                    }
                    // Set private occupancy measure
                    this->private_omap_[ag][jhist->getIndividualHistory(ag)].addProbabilityAt(state, std::make_shared<typename private_ostate_type::jhistory_type::element_type>(other_histories), proba);
                }
            }
        }
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
            return std::hash<sdm::BaseOccupancyState<S, V>>()(in);
        }
    };
}