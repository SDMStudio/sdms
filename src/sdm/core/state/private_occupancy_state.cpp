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
        for (const auto &pair_state_proba : *this)
        {
            auto history = std::static_pointer_cast<JointHistoryBeliefPair>(pair_state_proba.first)->first;
            auto belief = std::static_pointer_cast<JointHistoryBeliefPair>(pair_state_proba.first)->second;

            res << "\t<probability";
            res << " history=" << history->short_str() << "";
            res << " belief=" << belief->str() << ">\n";
            res << "\t\t\t" << pair_state_proba.second << "\n";
            res << "\t</probability \n";
        }
        res << "</private-occupancy-state>";
        return res.str();
    }

    std::vector<std::shared_ptr<HistoryInterface>> PrivateOccupancyState::getPartialJointHistory(const std::vector<std::shared_ptr<HistoryInterface>> &joint_history) const
    {
        auto partial_jhist = joint_history;
        partial_jhist.erase(partial_jhist.begin() + this->getAgentId());
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

    void PrivateOccupancyState::finalize(bool do_compression)
    {
        if (do_compression)
        {
            OccupancyState::finalize();
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
        // std::cout << "-------------- EQUIVALENCE -------------" << std::endl;
        // std::cout << *this << std::endl;
        // std::cout << "VERSUS" << std::endl;
        // std::cout << other << std::endl;
        // std::cout << "----------------------------------------" << std::endl;
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

            // Doing this way should be deleted in further versions (it is time consuming)
            std::unordered_map<std::shared_ptr<State>, double> sum_this_belief, sum_other_belief;
            for (const auto &belief : this->getBeliefsAt(current_joint_history))
            {
                for (const auto &state : belief->getStates())
                {
                    // Build value in the current private occupancy state
                    sum_this_belief[state] += this->getProbability(current_joint_history, belief, state);

                    // Get corresponding value in the other private occupancy state -----------> !!!!!!!!!!!! ATTENTION: Ici make_shared ne donnera pas la bonne addresse
                    sum_other_belief[state] += other.getProbability(other_joint_history, belief, state);

                    // const auto &current_value = this->getProbability(current_joint_history, belief, state);
                    // const auto &other_value = other.getProbability(other_joint_history, belief, state);
                }
            }

            // Check equivalence
            for (const auto &pair_state_value : sum_this_belief)
            {
                // std::cout << "std::abs(" << pair_state_value.second << " - " << sum_other_belief[pair_state_value.first] << ") = " << std::abs(pair_state_value.second - sum_other_belief[pair_state_value.first]) << " > " << PrivateOccupancyState::PRECISION_COMPRESSION << " ? " << (std::abs(pair_state_value.second - sum_other_belief[pair_state_value.first]) > PrivateOccupancyState::PRECISION_COMPRESSION) << std::endl;
                if (std::abs(pair_state_value.second - sum_other_belief[pair_state_value.first]) > PrivateOccupancyState::PRECISION_COMPRESSION)
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
