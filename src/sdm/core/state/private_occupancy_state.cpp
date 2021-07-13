#include <iomanip>
#include <sdm/config.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>

namespace sdm
{

    double PrivateOccupancyState::PRECISION_COMPRESSION = config::PRECISION_COMPRESSION;

    PrivateOccupancyState::PrivateOccupancyState()
    {
    }

    PrivateOccupancyState::PrivateOccupancyState(number num_agents) : OccupancyState(num_agents)
    {
    }

    PrivateOccupancyState::PrivateOccupancyState(number agent_id, number num_agents) : OccupancyState(num_agents), agent_id_(agent_id)
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

    std::string PrivateOccupancyState::str() const
    {
        std::ostringstream res;
        res << std::setprecision(config::OCCUPANCY_DECIMAL_PRINT) << std::fixed;

        res << "<private-occupancy-state agent=\"" << this->agent_id_ << "\" size=\"" << MappedVector<std::shared_ptr<State>>::size() << "\">\n";
        for (const auto &history_as_state : this->getIndexes())
        {
            auto joint_history = history_as_state->toHistory()->toJointHistory();

            res << "\t<probability";
            res << " history=" << joint_history->short_str() << "";
            res << " belief=" << this->getBeliefAt(joint_history)->str() << ">\n";
            res << "\t\t\t" << this->getProbability(joint_history) << "\n";
            res << "\t</probability \n";
        }
        res << "</private-occupancy-state>";
        return res.str();
    }

    std::vector<std::shared_ptr<HistoryInterface>> PrivateOccupancyState::getPartialJointHistory(const std::vector<std::shared_ptr<HistoryInterface>> &joint_history) const
    {
        // Copy full joint history
        auto partial_jhist = joint_history;
        
        // Erase the component associated to the agent
        partial_jhist.erase(partial_jhist.begin() + this->getAgentId());

        // Return the partial joint history
        return partial_jhist;
    }

    const std::vector<std::shared_ptr<HistoryInterface>> &PrivateOccupancyState::getPartialJointHistory(const std::shared_ptr<JointHistoryInterface> &joint_history) const
    {
        return bimap_jhist_partial_jhist.left.at(joint_history);
    }

    std::shared_ptr<JointHistoryInterface> PrivateOccupancyState::getJointHistory(const std::vector<std::shared_ptr<HistoryInterface>> &partial_joint_history) const
    {
        return bimap_jhist_partial_jhist.right.at(partial_joint_history);
    }

    void PrivateOccupancyState::finalize()
    {
        OccupancyState::finalize();
    }

    void PrivateOccupancyState::finalize(bool do_compression)
    {
        if (do_compression)
        {
            this->finalize();
        }
        else
        {
            this->setup();
        }

        // Add elements in bimap jhistory <--> jhistory^{-i}
        for (const auto &joint_history : this->getJointHistories())
        {
            // Get partial joint history
            const auto &partial_jhist = this->getPartialJointHistory(joint_history->getIndividualHistories());
            //
            this->bimap_jhist_partial_jhist.insert(bimap_value(joint_history, partial_jhist));
        }
    }

    bool PrivateOccupancyState::check_equivalence(const PrivateOccupancyState &other) const
    {
        // Check that private occupancy states are defined on the same support
        if (this->size() != other.size())
        {
            return false;
        }
        // Go over all partial joint histories and associated joint history
        for (const auto &pair_partial_joint_history_joint_history : this->bimap_jhist_partial_jhist.right)
        {
            const auto &partial_joint_history = pair_partial_joint_history_joint_history.first;
            const auto &current_joint_history = pair_partial_joint_history_joint_history.second;

            // Get an iterator on the first partial joint history that is similar in "other"
            auto iterator = other.bimap_jhist_partial_jhist.right.find(partial_joint_history);
            if (iterator == other.bimap_jhist_partial_jhist.right.end())
            {
                return false;
            }

            // Get the associated joint history of the second value
            const auto &other_joint_history = iterator->second;

            // For all states in the corresponding belief
            for (const auto &state : this->getBeliefAt(current_joint_history)->getStates())
            {
                // Compare p(o^{-i}, x | o^{i}_1) and p(o^{-i}, x | o^{i}_2)
                if (std::abs(this->getProbability(current_joint_history, state) - other.getProbability(other_joint_history, state)) > PrivateOccupancyState::PRECISION_COMPRESSION)
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
            return std::hash<sdm::MappedVector<std::shared_ptr<sdm::State>>>()(in, sdm::OccupancyState::PRECISION);
        }
    };
}
