#include <sdm/core/state/private_occupancy_state.hpp>

namespace sdm
{
    template <typename TState, typename TJointHistory_p>
    double PrivateOccupancyState<TState, TJointHistory_p>::PRECISION = OccupancyState<TState, TJointHistory_p>::PRECISION;

    template <typename TState, typename TJointHistory_p>
    PrivateOccupancyState<TState, TJointHistory_p>::PrivateOccupancyState()
    {
    }

    template <typename TState, typename TJointHistory_p>
    PrivateOccupancyState<TState, TJointHistory_p>::PrivateOccupancyState(
        double default_value) : OccupancyState<TState, TJointHistory_p>(default_value)
    {
    }

    template <typename TState, typename TJointHistory_p>
    PrivateOccupancyState<TState, TJointHistory_p>::PrivateOccupancyState(
        number num_agents, double default_value) : OccupancyState<TState, TJointHistory_p>(num_agents, default_value)
    {
    }

    template <typename TState, typename TJointHistory_p>
    PrivateOccupancyState<TState, TJointHistory_p>::PrivateOccupancyState(number agent_id, number num_agents, double default_value) : OccupancyState<TState, TJointHistory_p>(num_agents, default_value), agent_id_(agent_id)
    {
    }

    template <typename TState, typename TJointHistory_p>
    PrivateOccupancyState<TState, TJointHistory_p>::PrivateOccupancyState(
        const PrivateOccupancyState &v) : OccupancyState<TState, TJointHistory_p>(v), agent_id_(v.getAgentId()), bimap_jhist_partial_jhist(v.bimap_jhist_partial_jhist)
    {
    }

    template <typename TState, typename TJointHistory_p>
    PrivateOccupancyState<TState, TJointHistory_p>::PrivateOccupancyState(const OccupancyState<TState, TJointHistory_p> &occupancy_state)
        : OccupancyState<TState, TJointHistory_p>(occupancy_state),
          agent_id_(0),
          bimap_jhist_partial_jhist()
    {
    }

    template <typename TState, typename TJointHistory_p>
    number PrivateOccupancyState<TState, TJointHistory_p>::getAgentId() const
    {
        return this->agent_id_;
    }

    // template <typename TState, typename TJointHistory_p>
    // std::string PrivateOccupancyState<TState, TJointHistory_p>::str() const
    // {
    //     std::ostringstream res, tmp;
    //     res << "<private-occupancy-state horizon='?'>" << std::endl;
    //     for (const auto &pair_x_o_p : *this)
    //     {
    //         auto joint_hist = pair_x_o_p.first.second;

    //         res << "\t<probability state=\"" << pair_x_o_p.first.first << "\">" << std::endl;
    //         for (auto ihist : pair_x_o_p.first.second->getIndividualHistories())
    //         {
    //             res << tools::addIndent(ihist->str(), 2);
    //         }
    //         res << "\t\t" << pair_x_o_p.second << std::endl;
    //         res << "\t<probability>" << std::endl;
    //     }
    //     res << "</private-occupancy-state>" << std::endl;

    //     return res.str();
    // }

    template <typename TState, typename TJointHistory_p>
    std::vector<typename PrivateOccupancyState<TState, TJointHistory_p>::ihistory_type> PrivateOccupancyState<TState, TJointHistory_p>::getPartialJointHistory(const std::vector<typename PrivateOccupancyState<TState, TJointHistory_p>::ihistory_type> &joint_history) const
    {
        auto partial_jhist = joint_history;
        partial_jhist.erase(partial_jhist.begin() + this->getAgentId());
        return partial_jhist;
    }

    template <typename TState, typename TJointHistory_p>
    const std::vector<typename PrivateOccupancyState<TState, TJointHistory_p>::ihistory_type> &PrivateOccupancyState<TState, TJointHistory_p>::getPartialJointHistory(const TJointHistory_p &joint_history) const
    {
        return bimap_jhist_partial_jhist.left.at(joint_history);
    }

    template <typename TState, typename TJointHistory_p>
    TJointHistory_p PrivateOccupancyState<TState, TJointHistory_p>::getJointHistory(const std::vector<ihistory_type> &partial_joint_history) const
    {
        return bimap_jhist_partial_jhist.right.at(partial_joint_history);
    }

    template <typename TState, typename TJointHistory_p>
    void PrivateOccupancyState<TState, TJointHistory_p>::finalize(bool do_compression)
    {
        if (do_compression)
        {
            OccupancyState<TState, TJointHistory_p>::finalize();
        }
        else
        {
            BaseOccupancyState<TState, TJointHistory_p>::finalize();
        }

        // Add elements in bimap jhistory <--> jhistory^{-i}
        for (const auto &pair_state_hist_prob : *this)
        {
            const auto &jhist = this->getHistory(pair_state_hist_prob.first);
            // Get partial joint history
            const auto &partial_jhist = this->getPartialJointHistory(jhist->getIndividualHistories());
            //
            this->bimap_jhist_partial_jhist.insert(bimap_value(jhist, partial_jhist));
        }
    }

    template <typename TState, typename TJointHistory_p>
    bool PrivateOccupancyState<TState, TJointHistory_p>::operator==(const PrivateOccupancyState<TState, TJointHistory_p> &other) const
    {
        double ratio = -1;
        // Check that private occupancy states are defined on the same support
        if (this->size() != other.size())
        {
            return false;
        }
        for (const auto &pair_partial_joint_history_joint_history : this->bimap_jhist_partial_jhist.right)
        {
            const auto &partial_joint_history = pair_partial_joint_history_joint_history.first;
            const auto &current_joint_history = pair_partial_joint_history_joint_history.second;
            const auto &other_joint_history = other.bimap_jhist_partial_jhist.right.at(partial_joint_history);

            for (const auto &hidden_state : this->getStatesAt(current_joint_history))
            {
                // Get value in the current private occupancy state
                const auto &current_pair_state_joint_history = std::make_pair(hidden_state, current_joint_history);
                const auto &current_value = this->at(current_pair_state_joint_history);

                // Get corresponding value in the other private occupancy state
                const auto &other_pair_state_joint_history = std::make_pair(hidden_state, other_joint_history);
                const auto &other_value = other.at(other_pair_state_joint_history);

                if (other_value == 0)
                {
                    return false;
                }
                if (ratio < 0)
                {
                    ratio = current_value / other_value;
                }
                else if (std::abs(ratio - current_value / other_value) > PrivateOccupancyState::PRECISION)
                {
                    return false;
                }
            }
        }
        return true;
    }

} // namespace sdm

namespace std
{
    template <typename S, typename V>
    struct hash<sdm::PrivateOccupancyState<S, V>>
    {
        typedef sdm::PrivateOccupancyState<S, V> argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type &in) const
        {
            return std::hash<sdm::OccupancyState<S, V>>()(in);
        }
    };
}
