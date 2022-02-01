#include <iomanip>
#include <sdm/config.hpp>
#include <sdm/exception.hpp>
#include <sdm/core/state/compressed_occupancy_state.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>

namespace sdm
{
    double CompressedOccupancyState::PRECISION = config::PRECISION_OCCUPANCY_STATE;

    CompressedOccupancyState::CompressedOccupancyState() : CompressedOccupancyState(2)
    {
    }

    CompressedOccupancyState::CompressedOccupancyState(number num_agents) : OccupancyState(num_agents)
    {
    }

    CompressedOccupancyState::CompressedOccupancyState(const CompressedOccupancyState &occupancy_state)
    {
        throw sdm::exception::NotImplementedException();
    }

    CompressedOccupancyState::~CompressedOccupancyState()
    {
    }

    // #############################################
    // ###### MANIPULATE REPRESENTATION ############
    // #############################################

    const Joint<RecursiveMap<std::shared_ptr<HistoryInterface>, std::shared_ptr<PrivateOccupancyState>>> &CompressedOccupancyState::getPrivateOccupancyStates() const
    {
        return this->tuple_of_maps_from_histories_to_private_occupancy_states_;
    }

    const std::shared_ptr<PrivateOccupancyState> &CompressedOccupancyState::getPrivateOccupancyState(const number &agent_id, const std::shared_ptr<HistoryInterface> &ihistory) const
    {
        return this->tuple_of_maps_from_histories_to_private_occupancy_states_.at(agent_id).at(ihistory);
    }

    std::shared_ptr<OccupancyStateInterface> CompressedOccupancyState::getFullyUncompressedOccupancy()
    {
        // If fully uncompressed is nullptr then we consider fully uncompressed is equal to this
        return (this->fully_uncompressed_occupancy_state) ? this->fully_uncompressed_occupancy_state : this->getptr();
    }

    void CompressedOccupancyState::setFullyUncompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &fully_uncompressed_ostate)
    {
        if (fully_uncompressed_ostate != this->getptr())
            this->fully_uncompressed_occupancy_state = fully_uncompressed_ostate;
    }

    std::shared_ptr<OccupancyStateInterface> CompressedOccupancyState::getOneStepUncompressedOccupancy()
    {
        // If one step uncompressed is nullptr then we consider one step uncompressed is equal to this
        return (this->one_step_left_compressed_occupancy_state) ? this->one_step_left_compressed_occupancy_state : this->getptr();
    }

    void CompressedOccupancyState::setOneStepUncompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &one_step_uncompress_ostate)
    {
        if (one_step_uncompress_ostate != this->getptr())
            this->one_step_left_compressed_occupancy_state = one_step_uncompress_ostate;
        // std::dynamic_pointer_cast<CompressedOccupancyState>(this->getOneStepUncompressedOccupancy())->setCompressedOccupancy(this->getptr());
    }

    std::shared_ptr<OccupancyStateInterface> CompressedOccupancyState::getCompressedOccupancy()
    {
        return (!this->compressed_occupancy_state.expired()) ? this->compressed_occupancy_state.lock() : this->getptr();
    }

    void CompressedOccupancyState::setCompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &compress_ostate)
    {
        this->compressed_occupancy_state = compress_ostate;
    }

    // #####################################
    // ###### MANIPULATE LABELS ############
    // #####################################

    std::shared_ptr<HistoryInterface> CompressedOccupancyState::getLabel(const std::shared_ptr<HistoryInterface> &ihistory, number agent_id) const
    {
        auto iterator = this->private_ihistory_map_.at(agent_id).find(ihistory);
        return (iterator == this->private_ihistory_map_.at(agent_id).end()) ? ihistory : iterator->second;
    }

    Joint<std::shared_ptr<HistoryInterface>> CompressedOccupancyState::getJointLabels(const Joint<std::shared_ptr<HistoryInterface>> &list_ihistories) const
    {
        Joint<std::shared_ptr<HistoryInterface>> new_list_ihistories;
        for (int agent_id = 0; agent_id < this->num_agents_; ++agent_id)
        {
            // if the ihistory was never compressed
            new_list_ihistories.push_back(this->getLabel(list_ihistories.at(agent_id), agent_id));
        }
        return new_list_ihistories;
    }

    void CompressedOccupancyState::updateLabel(number agent_id, const std::shared_ptr<HistoryInterface> &ihistory, const std::shared_ptr<HistoryInterface> &label)
    {
        this->private_ihistory_map_[agent_id][ihistory] = label;
        if (ihistory != label)
        {
            for (const auto &pair_ihistory_label : this->private_ihistory_map_[agent_id])
            {
                if (pair_ihistory_label.second == ihistory)
                {
                    this->updateLabel(agent_id, pair_ihistory_label.first, label);
                }
            }
        }
    }

    void CompressedOccupancyState::updateJointLabels(const Joint<std::shared_ptr<HistoryInterface>> &list_ihistories, const Joint<std::shared_ptr<HistoryInterface>> &list_labels)
    {
        for (number agent_id = 0; agent_id < this->num_agents_; ++agent_id)
        {
            this->updateLabel(agent_id, list_ihistories.at(agent_id), list_labels.at(agent_id));
        }
    }

    Pair<std::shared_ptr<State>, double> CompressedOccupancyState::nextStateAndProba(const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        // The new one step left occupancy state
        std::shared_ptr<OccupancyStateInterface> next_one_step_left_compressed_occupancy_state = std::make_shared<OccupancyState>(this->num_agents_);

        // For each joint history in the support of the fully uncompressed occupancy state
        for (const auto &compressed_joint_history : this->getJointHistories())
        {
            // Get p(o_t)
            double proba_history = this->getProbability(compressed_joint_history);

            // Get the corresponding belief
            auto belief = this->getBeliefAt(compressed_joint_history);

            // Apply decision rule and get action
            auto jaction = oMDP->applyDecisionRule(compressed_occupancy_state, compressed_joint_history, decision_rule, t);

            // For each action that is likely to be taken
            for (const auto &joint_action : {jaction}) // decision_rule->getDistribution(compressed_joint_history)->getSupport())
            {
                // Get p(u_t | o_t)
                double proba_action = 1.0; // decision_rule->getProbability(compressed_joint_history, joint_action);

                // For each observation in the space of joint observation
                for (auto jobs : *oMDP->getUnderlyingMPOMDP()->getObservationSpace(t))
                {
                    auto joint_observation = jobs->toObservation();
                    if (this->checkCompatibility(joint_observation, observation))
                    {
                        // Get the next belief and p(z_{t+1} | b_t, u_t)
                        auto [next_belief, proba_observation] = oMDP->getUnderlyingBeliefMDP()->getNextStateAndProba(belief, joint_action, joint_observation, t);

                        double next_joint_history_probability = proba_history * proba_action * proba_observation;

                        // If the next history probability is not zero
                        if (next_joint_history_probability > 0)
                        {
                            // Update new one step uncompressed occupancy state
                            std::shared_ptr<JointHistoryInterface> next_compressed_joint_history = compressed_joint_history->expand(joint_observation /*, joint_action*/)->toJointHistory();
                            oMDP->updateOccupancyStateProba(next_one_step_left_compressed_occupancy_state, next_compressed_joint_history, next_belief->toBelief(), next_joint_history_probability);
                        }
                    }
                }
            }
        }

        return oMDP->finalizeNextState(next_one_step_left_compressed_occupancy_state, nullptr, t);
    }

    // #############################################
    // ######### MANIPULATE COMPRESSION ############
    // #############################################

    std::shared_ptr<JointHistoryInterface> CompressedOccupancyState::getCompressedJointHistory(const std::shared_ptr<JointHistoryInterface> &joint_history) const
    {
        const auto &labels = this->getJointLabels(joint_history->getIndividualHistories());
        return this->jhistory_map_.at(labels);
    }

    bool CompressedOccupancyState::areIndividualHistoryLPE(const std::shared_ptr<HistoryInterface> &ihistory_1, const std::shared_ptr<HistoryInterface> &ihistory_2, number agent_identifier)
    {
        return this->getPrivateOccupancyState(agent_identifier, ihistory_1)->check_equivalence(*this->getPrivateOccupancyState(agent_identifier, ihistory_2));
    }

    /**
     * @brief
     *
     * https://gitlab.inria.fr/maintenance/maintenance.html?appli=GITLAB
     * @return std::shared_ptr<OccupancyStateInterface>
     */
    std::shared_ptr<OccupancyStateInterface> CompressedOccupancyState::compress()
    {

        auto current_compact_ostate = std::make_shared<CompressedOccupancyState>(this->num_agents_);
        auto previous_compact_ostate = std::make_shared<CompressedOccupancyState>(*this);

        for (int agent_id = 0; agent_id < this->num_agents_; ++agent_id)
        {
            // Get support (a set of individual histories for agent i)
            const auto &support_set = this->getIndividualHistories(agent_id);
            auto &&support = tools::set2vector(support_set);

            // Sort support
            std::sort(support.begin(), support.end());

            for (auto iter_first = support.begin(); iter_first != support.end();)
            {
                auto ihistory_label = *iter_first;      // Get the ihistory "label"
                iter_first = support.erase(iter_first); // Erase the ihistory "label" from the support

                // Set probability of labels
                for (const auto &joint_history : previous_compact_ostate->getPrivateOccupancyState(agent_id, ihistory_label)->getJointHistories())
                {
                    auto belief = previous_compact_ostate->getPrivateOccupancyState(agent_id, ihistory_label)->getBeliefAt(joint_history);
                    current_compact_ostate->setProbability(joint_history, belief, previous_compact_ostate->getProbability(joint_history));
                }

                // For all other individual histories in the support
                for (auto iter_second = iter_first; iter_second != support.end();)
                {
                    // Get the ihistory we want check the equivalence
                    auto ihistory_one_step_left = *iter_second;

                    // Check equivalence between individual histories
                    if (this->areIndividualHistoryLPE(ihistory_label, ihistory_one_step_left, agent_id))
                    {
                        // If ihistories are equivalent
                        // Store the new label
                        this->updateLabel(agent_id, ihistory_one_step_left, ihistory_label);

                        // Erase unecessary equivalent individual history
                        iter_second = support.erase(iter_second);

                        // ----- Update probability of the new compact occupancy state by adding proba of the equivalent ihistory ---
                        // For all private joint history in the previous compact occupancy state
                        for (const auto &private_joint_history : previous_compact_ostate->getPrivateOccupancyState(agent_id, ihistory_one_step_left)->getJointHistories())
                        {
                            // Get the probability of the private occupancy state corresponding to the history that will be deleted
                            double probability = this->weight_of_private_occupancy_state_[agent_id][ihistory_one_step_left] * previous_compact_ostate->getPrivateOccupancyState(agent_id, ihistory_one_step_left)->getProbability(private_joint_history);

                            // Get the partial joint history label
                            auto partial_jhist = previous_compact_ostate->getPrivateOccupancyState(agent_id, ihistory_one_step_left)->getPartialJointHistory(private_joint_history);

                            // Get the joint history corresponding to this partial joint history in the private occupancy state of the history label
                            auto joint_history_from_partial = previous_compact_ostate->getPrivateOccupancyState(agent_id, ihistory_label)->getJointHistoryFromPartial(partial_jhist);

                            // Update the current compact occupancy state
                            current_compact_ostate->addProbability(joint_history_from_partial,
                                                                   previous_compact_ostate->getPrivateOccupancyState(agent_id, ihistory_label)->getBeliefAt(joint_history_from_partial),
                                                                   probability);
                        }
                    }
                    else
                    {
                        iter_second++;
                    }
                }
            }

            *previous_compact_ostate = *current_compact_ostate;
            previous_compact_ostate->private_ihistory_map_ = this->private_ihistory_map_;
            previous_compact_ostate->finalize();
            current_compact_ostate->clear();
        }

        // previous_compact_ostate->setFullyUncompressedOccupancy(this->getFullyUncompressedOccupancy());
        // previous_compact_ostate->setOneStepUncompressedOccupancy(this->getptr());

        return previous_compact_ostate;
    }

    void CompressedOccupancyState::setupPrivateOccupancyStates()
    {
        // For all joint histories in the support of the occupancy state
        for (const auto &jhist : this->getJointHistories())
        {
            // Get the probability of being in this history
            const auto &proba = this->getProbability(jhist);

            // Get the corresponding belief
            auto belief = this->getBeliefAt(jhist);

            // Store relation between joint history and list of individual histories
            this->jhistory_map_.emplace(jhist->getIndividualHistories(), jhist);

            // For each agent we update its private occupancy state
            for (number agent_id = 0; agent_id < this->num_agents_; agent_id++)
            {
                // Instanciation empty private occupancy state associated to ihistory and agent i if not exists
                if (this->tuple_of_maps_from_histories_to_private_occupancy_states_[agent_id].find(jhist->getIndividualHistory(agent_id)) == this->tuple_of_maps_from_histories_to_private_occupancy_states_[agent_id].end())
                {
                    this->tuple_of_maps_from_histories_to_private_occupancy_states_[agent_id].emplace(jhist->getIndividualHistory(agent_id), std::make_shared<PrivateOccupancyState>(agent_id, this->num_agents_));
                }
                // Set private occupancy measure
                this->tuple_of_maps_from_histories_to_private_occupancy_states_[agent_id][jhist->getIndividualHistory(agent_id)]->addProbability(jhist, belief, proba);
            }
        }

        // ----------- FINALIZE PRIVATE OCCUPANCY STATES --------------
        // For all agents
        for (number agent_id = 0; agent_id < this->num_agents_; agent_id++)
        {
            // For all individual histories
            for (const auto &pair_ihist_private_occupancy_state : this->tuple_of_maps_from_histories_to_private_occupancy_states_[agent_id])
            {
                // Finalize the private occupancy state
                pair_ihist_private_occupancy_state.second->finalize(false);

                // Get the weight of a private occupancy state
                this->weight_of_private_occupancy_state_[agent_id][pair_ihist_private_occupancy_state.first] = pair_ihist_private_occupancy_state.second->norm_1();

                // Normalize the private occupancy state
                pair_ihist_private_occupancy_state.second->normalize();
            }
        }
    }

} // namespace sdm
