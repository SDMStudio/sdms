#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>
#include <sdm/exception.hpp>

namespace sdm
{
    double OccupancyState::PRECISION = config::PRECISION_OCCUPANCY_STATE;

    OccupancyState::OccupancyState(double default_value) : OccupancyState(2, default_value)
    {
    }

    OccupancyState::OccupancyState(number num_agents, double default_value) : Belief(default_value), num_agents_(num_agents)
    {
        for (number agent_id = 0; agent_id < num_agents; agent_id++)
        {
            this->tuple_of_maps_from_histories_to_private_occupancy_states_.push_back({});
            this->private_ihistory_map_.push_back({});
            this->map_label_to_pointer.push_back({});
        }
    }

    OccupancyState::OccupancyState(const OccupancyState &occupancy_state)
        : MappedVector<std::shared_ptr<BaseState<Pair<std::shared_ptr<State>, std::shared_ptr<JointHistoryInterface>>>>, double>(occupancy_state),
          tuple_of_maps_from_histories_to_private_occupancy_states_(occupancy_state.tuple_of_maps_from_histories_to_private_occupancy_states_),
          fully_uncompressed_occupancy_state(occupancy_state.fully_uncompressed_occupancy_state),
          one_step_left_compressed_occupancy_state(occupancy_state.one_step_left_compressed_occupancy_state),
          private_ihistory_map_(occupancy_state.private_ihistory_map_),
          map_label_to_pointer(occupancy_state.map_label_to_pointer),
          jhistory_map_(occupancy_state.jhistory_map_),
          probability_ihistories(occupancy_state.probability_ihistories),
          list_states(occupancy_state.list_states),
          list_jhistories(occupancy_state.list_jhistories),
          list_jhistory_states(occupancy_state.list_jhistory_states),
          num_agents_(occupancy_state.num_agents_),
          all_list_ihistories(occupancy_state.all_list_ihistories),
          ihistories_to_jhistory(occupancy_state.ihistories_to_jhistory),
          probability_jhistories(occupancy_state.probability_jhistories)
    {
    }

    // ###################################
    // ###### MANIPULATE DATA ############
    // ###################################

    const std::set<std::shared_ptr<JointHistoryInterface>> &getJointHistories() const
    {
        return this->list_joint_histories_;
    }

    const std::shared_ptr<BeliefInterface> &getBeliefAt(const std::shared_ptr<JointHistoryInterface> &jhistory) const
    {
        return this->map_joint_history_to_belief_.at(jhistory);
    }

    const std::set<std::shared_ptr<HistoryInterface>> &getIndividualHistories(number agent_id) const
    {
        return this->all_list_ihistories[agent_id];
    }

    const std::vector<std::set<std::shared_ptr<HistoryInterface>>> &getAllIndividualHistories() const
    {
        return this->all_list_ihistories;
    }

    TypeState OccupancyState::getTypeState() const
    {
        return TypeState::OCCUPANCY_STATE;
    }

    void finalize()
    {
    }

    // #############################################
    // ###### MANIPULATE REPRESENTATION ############
    // #############################################

    const Joint<RecursiveMap<std::shared_ptr<HistoryInterface>, std::shared_ptr<PrivateOccupancyState>>> &OccupancyState::getPrivateOccupancyStates() const
    {
        return this->tuple_of_maps_from_histories_to_private_occupancy_states_;
    }

    const std::shared_ptr<PrivateOccupancyState> &OccupancyState::getPrivateOccupancyState(const number &agent_id, const std::shared_ptr<HistoryInterface> &ihistory) const
    {
        return this->tuple_of_maps_from_histories_to_private_occupancy_states_.at(agent_id).at(ihistory);
    }

    std::shared_ptr<OccupancyStateInterface> OccupancyState::getFullyUncompressedOccupancy() const
    {
        return this->fully_uncompressed_occupancy_state;
    }

    void OccupancyState::setFullyUncompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &fully_uncompressed_ostate)
    {
        this->fully_uncompressed_occupancy_state = fully_uncompressed_ostate;
    }

    std::shared_ptr<OccupancyStateInterface> OccupancyState::getOneStepUncompressedOccupancy() const
    {
        return this->one_step_left_compressed_occupancy_state;
    }

    void OccupancyState::setOneStepUncompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &one_step_uncompress_ostate)
    {
        this->one_step_left_compressed_occupancy_state = one_step_uncompress_ostate;
        std::static_pointer_cast<OccupancyState>(one_step_uncompress_ostate)->setCompressedOccupancy(this->getptr());
    }

    std::shared_ptr<OccupancyStateInterface> OccupancyState::getCompressedOccupancy() const
    {
        return this->compressed_occupancy_state;
    }

    void OccupancyState::setCompressedOccupancy(const std::shared_ptr<OccupancyStateInterface> &compress_ostate)
    {
        this->compressed_occupancy_state = compress_ostate;
    }

    // #####################################
    // ###### MANIPULATE LABELS ############
    // #####################################

    std::shared_ptr<HistoryInterface> OccupancyState::getLabel(const std::shared_ptr<HistoryInterface> &ihistory, number agent_id) const
    {
        if (this->private_ihistory_map_.at(agent_id).find(ihistory) == this->private_ihistory_map_.at(agent_id).end())
        {
            // if the ihistory was never compressed
            return ihistory;
        }
        else
        {
            // if the ihistory was compressed
            return this->private_ihistory_map_.at(agent_id).at(ihistory);
        }
    }

    std::vector<std::shared_ptr<HistoryInterface>> getJointLabels(const std::vector<std::shared_ptr<HistoryInterface>> &) const
    {
        std::vector<std::shared_ptr<HistoryInterface>> new_list_ihistories;
        for (int agent_id = 0; agent_id < this->num_agents_; ++agent_id)
        {
            // if the ihistory was never compressed
            new_list_ihistories.push_back(this->getLabel(list_ihistories.at(agent_id), agent_id));
        }
        return new_list_ihistories;
    }

    void OccupancyState::updateLabel(number agent_id, const std::shared_ptr<HistoryInterface> &ihistory, const std::shared_ptr<HistoryInterface> &label)
    {
        // if there is a label for ihistory
        if (this->private_ihistory_map_[agent_id].find(ihistory) != this->private_ihistory_map_[agent_id].end())
        {
            // Get the old label
            auto &&old_label = this->private_ihistory_map_[agent_id].at(ihistory);
            // Change every labels of ihistories that have old_label as label

            old_label = label;
        }
        else
        {
            // Check if the label is already used for another indiv history
            if (this->map_label_to_pointer[agent_id].find(label) != this->map_label_to_pointer[agent_id].end())
            {
                this->private_ihistory_map_[agent_id][ihistory] = this->map_label_to_pointer[agent_id].at(label);
            }
            else
            {
                // If no such label is already used, create a pointer on it and store it
                auto &&new_ptr_on_label = label;
                this->map_label_to_pointer[agent_id][label] = new_ptr_on_label;
                this->private_ihistory_map_[agent_id][ihistory] = new_ptr_on_label;
            }
        }
    }

    void OccupancyState::updateJointLabels(const std::vector<std::shared_ptr<HistoryInterface>> &list_ihistories, const std::vector<std::shared_ptr<HistoryInterface>> &list_labels)
    {
        for (number agent_id = 0; agent_id < this->num_agents_; ++agent_id)
        {
            this->updateLabel(agent_id, list_ihistories.at(agent_id), list_labels.at(agent_id));
        }
    }

    // #############################################
    // ######### MANIPULATE COMPRESSION ############
    // #############################################

    std::shared_ptr<JointHistoryInterface> OccupancyState::getCompressedJointHistory(const std::shared_ptr<JointHistoryInterface> &joint_history) const
    {
        const auto &labels = this->getJointLabels(joint_history->getIndividualHistories());
        return this->jhistory_map_.at(labels);
    }

    bool OccupancyState::areIndividualHistoryLPE(const std::shared_ptr<HistoryInterface> &ihistory_1, const std::shared_ptr<HistoryInterface> &ihistory_2, number agent_identifier)
    {
        return this->getPrivateOccupancyState(agent_identifier, ihistory_1) == this->getPrivateOccupancyState(agent_identifier, ihistory_2);
    }

    std::shared_ptr<OccupancyStateInterface> OccupancyState::compress()
    {
        OccupancyState current_compact_ostate(this->num_agents_);
        OccupancyState previous_compact_ostate = *this;

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
                for (const auto &pair_s_o_prob : *previous_compact_ostate.getPrivateOccupancyState(agent_id, ihistory_label))
                {
                    current_compact_ostate.setProbability(pair_s_o_prob.first, pair_s_o_prob.second);
                }
                for (auto iter_second = iter_first; iter_second != support.end();)
                {
                    auto ihistory_one_step_left = *iter_second; // Get the ihistory we want check the equivalence
                    if (this->areIndividualHistoryLPE(ihistory_label, ihistory_one_step_left, agent_id))
                    {
                        // Store new label
                        this->updateLabel(agent_id, ihistory_one_step_left, ihistory_label);

                        // Erase unecessary equivalent individual history
                        iter_second = support.erase(iter_second);
                        for (const auto &pair_s_o_prob : *previous_compact_ostate.getPrivateOccupancyState(agent_id, ihistory_one_step_left))
                        {
                            auto partial_jhist = previous_compact_ostate.getPrivateOccupancyState(agent_id, ihistory_one_step_left)->getPartialJointHistory(this->getHistory(pair_s_o_prob.first));
                            auto joint_history = previous_compact_ostate.getPrivateOccupancyState(agent_id, ihistory_label)->getJointHistory(partial_jhist);
                            current_compact_ostate.addProbability(this->getHiddenState(pair_s_o_prob.first), joint_history, pair_s_o_prob.second);
                        }
                    }
                    else
                    {
                        iter_second++;
                    }
                }
            }

            previous_compact_ostate = current_compact_ostate;
            previous_compact_ostate.private_ihistory_map_ = this->private_ihistory_map_;
            previous_compact_ostate.finalize();
            current_compact_ostate.clear();
        }
        return std::shared_ptr<OccupancyStateInterface>(std::make_shared<OccupancyState>(previous_compact_ostate));
    }

    void OccupancyState::finalize()
    {
        this->setStates();
        this->setJointHistories();
        this->setAllIndividualHistories();
        this->setJointHistoryOverIndividualHistories();
        this->setProbabilityOverJointHistory();

        for (const auto &jhist : this->getJointHistories())
        {
            for (const auto &state : this->getStatesAt(jhist))
            {
                const auto &proba = this->getProbability(state, jhist);

                // Store relation between joint history and list of individual histories
                this->jhistory_map_.emplace(jhist->getIndividualHistories(), jhist);

                // For each agent we update its private occupancy state
                for (number agent_id = 0; agent_id < this->num_agents_; agent_id++)
                {
                    // std::cout << "agent_id=" << agent_id << std::endl;
                    // Instanciation empty private occupancy state associated to ihistory and agent i if not exists
                    if (this->tuple_of_maps_from_histories_to_private_occupancy_states_[agent_id].find(jhist->getIndividualHistory(agent_id)) == this->tuple_of_maps_from_histories_to_private_occupancy_states_[agent_id].end())
                    {
                        this->tuple_of_maps_from_histories_to_private_occupancy_states_[agent_id].emplace(jhist->getIndividualHistory(agent_id), std::make_shared<PrivateOccupancyState>(agent_id, this->num_agents_, this->default_value_));
                    }
                    // Set private occupancy measure
                    this->tuple_of_maps_from_histories_to_private_occupancy_states_[agent_id][jhist->getIndividualHistory(agent_id)]->addProbability(state, jhist, proba);
                }
            }
        }
        for (number agent_id = 0; agent_id < this->num_agents_; agent_id++)
        {
            for (const auto &pair_ihist_private_occupancy_state : this->tuple_of_maps_from_histories_to_private_occupancy_states_[agent_id])
            {
                pair_ihist_private_occupancy_state.second->finalize(false);
            }
        }

        this->setProbabilityOverIndividualHistories();
    }

    std::shared_ptr<OccupancyState> OccupancyState::getptr()
    {
        return std::static_pointer_cast<OccupancyState>(this->shared_from_this()->toState()->toOccupancyState());
    }
} // namespace sdm
