#include <sdm/core/state/private_occupancy_state.hpp>

namespace sdm
{
    PrivateOccupancyState::PrivateOccupancyState()
    {
    }

    PrivateOccupancyState::PrivateOccupancyState(
        double default_value) : OccupancyState(default_value)
    {
    }

    PrivateOccupancyState::PrivateOccupancyState(
        number num_agents, double default_value) : OccupancyState(num_agents, default_value)
    {
    }

    PrivateOccupancyState::PrivateOccupancyState(number agent_id, number num_agents, double default_value) : OccupancyState(num_agents, default_value), agent_id_(agent_id)
    {
    }

    PrivateOccupancyState::PrivateOccupancyState(const PrivateOccupancyState &v) 
        : OccupancyState(v),
          agent_id_(v.getAgentId()),
          bimap_jhist_partial_jhist(v.bimap_jhist_partial_jhist)
    {
    }

    PrivateOccupancyState::PrivateOccupancyState(const OccupancyState &occupancy_state)
        : OccupancyState(occupancy_state),
          agent_id_(0),
          bimap_jhist_partial_jhist()
    {
    }

    number PrivateOccupancyState::getAgentId() const
    {
        return this->agent_id_;
    }

    // template <typename TState, typename TJointHistory_p>
    // std::string PrivateOccupancyState::str() const
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

    std::vector<std::shared_ptr<HistoryTreeInterface>> PrivateOccupancyState::getPartialJointHistory(const std::vector<std::shared_ptr<HistoryTreeInterface>> &joint_history) const
    {
        auto partial_jhist = joint_history;
        partial_jhist.erase(partial_jhist.begin() + this->getAgentId());
        return partial_jhist;
    }

    const std::vector<std::shared_ptr<HistoryTreeInterface>> &PrivateOccupancyState::getPartialJointHistory(const std::shared_ptr<JointHistoryTreeInterface> &joint_history) const
    {
        return bimap_jhist_partial_jhist.left.at(joint_history);
    }

    std::shared_ptr<JointHistoryTreeInterface> PrivateOccupancyState::getJointHistory(const std::vector<std::shared_ptr<HistoryTreeInterface>> &partial_joint_history) const
    {
        return bimap_jhist_partial_jhist.right.at(partial_joint_history);
    }

    void PrivateOccupancyState::finalize(bool do_compression)
    {
        if (do_compression)
        {
            OccupancyState::finalize();
        }
        else
        {
            this->setStates();
            this->setJointHistories();
            this->setAllIndividualHistories();
            this->setJointHistoryOverIndividualHistories();
            this->setProbabilityOverJointHistory();
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

    bool PrivateOccupancyState::operator==(const PrivateOccupancyState &other) const
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

            auto iterator = other.bimap_jhist_partial_jhist.right.find(partial_joint_history);
            if (iterator == other.bimap_jhist_partial_jhist.right.end())
            {
                return false;
            }

            const auto &other_joint_history = iterator->second;

            for (const auto &hidden_state : this->getStatesAt(current_joint_history))
            {
                // Get value in the current private occupancy state
                const auto &current_pair_state_joint_history = std::make_shared<BaseState<Pair<std::shared_ptr<State>,std::shared_ptr<JointHistoryTreeInterface>>>>(std::make_pair(hidden_state, current_joint_history));
                const auto &current_value = this->at(current_pair_state_joint_history);

                // Get corresponding value in the other private occupancy state
                const auto &other_pair_state_joint_history = std::make_shared<BaseState<Pair<std::shared_ptr<State>,std::shared_ptr<JointHistoryTreeInterface>>>>(std::make_pair(hidden_state, other_joint_history));
                const auto &other_value = other.at(other_pair_state_joint_history);

                if (other_value == 0)
                {
                    return false;
                }
                if (ratio < 0)
                {
                    ratio = current_value / other_value;
                }
                else if (std::abs(ratio - current_value / other_value) > this->PRECISION)
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
    template <>
    struct hash<sdm::PrivateOccupancyState>
    {
        typedef sdm::PrivateOccupancyState argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type &in) const
        {
            return std::hash<sdm::PrivateOccupancyState>()(in);
        }
    };
}
