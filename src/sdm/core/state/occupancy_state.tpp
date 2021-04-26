#include <sdm/core/state/occupancy_state.hpp>

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
    OccupancyState<TState, TJointHistory_p>::OccupancyState(const OccupancyState &v) : BaseOccupancyState<TState, TJointHistory_p>(v)
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
        // for (const auto &pair_state_jhist : *this)
        // {
        //     auto state = this->getState(pair_state_jhist);
        //     auto jhist = this->getHistory(pair_state_jhist);
        //     auto proba = this->getProbability(pair_state_jhist);
        //     for (number ag = 0; ag < jhist->getNumAgents(); ag++)
        //     {
        //         auto other_histories = jhist->getIndividualHistories();
        //         other_histories.erase(std::remove(other_histories.begin(), other_histories.end(), jhist->getIndividualHistory(ag)), other_histories.end());
        //         this->private_omap_[ag][jhist->getIndividualHistory(ag)][{state, std::make_shared<TJointHistory_p>(other_histories)}] += proba;
        //     }
        // }
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