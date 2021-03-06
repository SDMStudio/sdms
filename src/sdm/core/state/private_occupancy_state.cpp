#include <iomanip>
#include <sdm/config.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>

namespace sdm
{

    double PrivateOccupancyState::PRECISION_COMPRESSION = config::PRECISION_COMPRESSION;

    PrivateOccupancyState::PrivateOccupancyState()
    {
    }

    PrivateOccupancyState::PrivateOccupancyState(number num_agents, number h) : OccupancyState(num_agents, h)
    {
    }

    PrivateOccupancyState::PrivateOccupancyState(number agent_id, number num_agents, number h) : OccupancyState(num_agents, h), agent_id_(agent_id)
    {
    }

    PrivateOccupancyState::PrivateOccupancyState(const PrivateOccupancyState &v)
        : OccupancyState(v),
          agent_id_(v.getAgentId()),
          map_jhist_to_partial(v.map_jhist_to_partial),
          map_partial_to_jhist(v.map_partial_to_jhist)
    {
    }

    PrivateOccupancyState::PrivateOccupancyState(const OccupancyState &occupancy_state)
        : OccupancyState(occupancy_state),
          agent_id_(0)
    {
    }

    number PrivateOccupancyState::getAgentId() const
    {
        return this->agent_id_;
    }

    std::string PrivateOccupancyState::str() const
    {
        std::ostringstream res;
        res << std::setprecision(config::OCCUPANCY_DECIMAL_PRINT) << std::fixed;

        res << "<private-occupancy-state agent=\"" << this->agent_id_ << "\" size=\"" << this->size() << "\">\n";
        for (const auto &history_as_state : this->getStates())
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
        return this->map_jhist_to_partial.at(joint_history);
    }

    std::shared_ptr<JointHistoryInterface> PrivateOccupancyState::getJointHistoryFromPartial(const std::vector<std::shared_ptr<HistoryInterface>> &partial_joint_history) const
    {
        return this->map_partial_to_jhist.at(partial_joint_history);
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
            this->map_partial_to_jhist[partial_jhist] = joint_history;
            this->map_jhist_to_partial[joint_history] = partial_jhist;
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
        for (const auto &pair_partial_jhistory : this->map_partial_to_jhist)
        {
            const auto &partial_joint_history = pair_partial_jhistory.first;
            const auto &current_joint_history = pair_partial_jhistory.second;

            // Get an iterator on the first partial joint history that is similar in "other"
            auto iterator = other.map_partial_to_jhist.find(partial_joint_history);
            if (iterator == other.map_partial_to_jhist.end())
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
