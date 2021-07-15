#include <iomanip>
#include <sdm/config.hpp>
#include <sdm/exception.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>

namespace sdm
{
    double OccupancyState::PRECISION = config::PRECISION_OCCUPANCY_STATE;
    double OccupancyState::TIME_IN_GET_PROBA = 0, OccupancyState::TIME_IN_SET_PROBA = 0, OccupancyState::TIME_IN_ADD_PROBA = 0, OccupancyState::TIME_IN_FINALIZE = 0, OccupancyState::TIME_IN_EQUAL_OPERATOR = 0;
    unsigned long OccupancyState::PASSAGE_GET_PROBA = 0, OccupancyState::PASSAGE_SET_PROBA = 0, OccupancyState::PASSAGE_FINALIZE = 0;

    OccupancyState::OccupancyState() : OccupancyState(2)
    {
    }

    OccupancyState::OccupancyState(number num_agents) : Belief(num_agents), num_agents_(num_agents), action_space_map(std::make_shared<std::unordered_map<number, std::shared_ptr<Space>>>())
    {
        for (number agent_id = 0; agent_id < num_agents; agent_id++)
        {
            this->tuple_of_maps_from_histories_to_private_occupancy_states_.push_back({});
            this->weight_of_private_occupancy_state_.push_back({});
            this->private_ihistory_map_.push_back({});
            this->map_label_to_pointer.push_back({});
            //
            this->individual_hierarchical_history_vector_map_vector.push_back(std::make_shared<std::unordered_map<number, std::vector<std::shared_ptr<JointHistoryInterface>>>>());
        }
    }

    OccupancyState::OccupancyState(const OccupancyState &occupancy_state)
        : Belief(occupancy_state),
          num_agents_(occupancy_state.num_agents_),
          tuple_of_maps_from_histories_to_private_occupancy_states_(occupancy_state.tuple_of_maps_from_histories_to_private_occupancy_states_),
          weight_of_private_occupancy_state_(occupancy_state.weight_of_private_occupancy_state_),
          fully_uncompressed_occupancy_state(occupancy_state.fully_uncompressed_occupancy_state),
          one_step_left_compressed_occupancy_state(occupancy_state.one_step_left_compressed_occupancy_state),
          compressed_occupancy_state(occupancy_state.compressed_occupancy_state),
          private_ihistory_map_(occupancy_state.private_ihistory_map_),
          map_label_to_pointer(occupancy_state.map_label_to_pointer),
          jhistory_map_(occupancy_state.jhistory_map_),
          probability_ihistories(occupancy_state.probability_ihistories),
          list_beliefs_(occupancy_state.list_beliefs_),
          list_joint_histories_(occupancy_state.list_joint_histories_),
          all_list_ihistories_(occupancy_state.all_list_ihistories_),
          map_joint_history_to_belief_(occupancy_state.map_joint_history_to_belief_),
          ihistories_to_jhistory_(occupancy_state.ihistories_to_jhistory_),
          action_space_map(std::make_shared<std::unordered_map<number, std::shared_ptr<Space>>>())
    {
    }

    double OccupancyState::getProbability(const std::shared_ptr<State> &joint_history) const
    {
        return Belief::getProbability(joint_history);
    }

    double OccupancyState::getProbability(const std::shared_ptr<JointHistoryInterface> &joint_history) const
    {
        return Belief::getProbability(joint_history);
    }

    double OccupancyState::getProbability(const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<State> &state) const
    {
        // Get the probability p(x,o) = p(o) * b(x | o)
        clock_t t_begin = clock();
        auto belief = this->getBeliefAt(joint_history);
        auto output = (belief == nullptr) ? this->getDefault() : this->getProbability(joint_history) * belief->getProbability(state);
        OccupancyState::TIME_IN_GET_PROBA += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);
        return output;
        // return this->getProbability(joint_history) * this->getBeliefAt(joint_history)->getProbability(state);
    }

    void OccupancyState::setProbability(const std::shared_ptr<State> &joint_history, double proba)
    {
        Belief::setProbability(joint_history, proba);
        std::cout << "-------------------ERROOOOOR --------------\n\n\n\n\n\n\n";
    }

    void OccupancyState::setProbability(const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<BeliefInterface> &belief, double proba)
    {
        clock_t t_begin = clock();
        // Set the belief corresponding to a specific joint history
        this->setBeliefAt(joint_history, belief);
        // Set the probability of the joint history
        Belief::setProbability(joint_history, proba);
        // this->setProbability(joint_history, proba);
        OccupancyState::TIME_IN_SET_PROBA += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);
    }

    void OccupancyState::addProbability(const std::shared_ptr<State> &joint_history, double proba)
    {
        // Add the probability of being in a joint history
        this->setProbability(joint_history, this->getProbability(joint_history) + proba);

        std::cout << "-------------------ERROOOOOR --------------\n\n\n\n\n\n\n";
    }

    void OccupancyState::addProbability(const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<BeliefInterface> &belief, double proba)
    {
        clock_t t_begin = clock();
        // Get the corresponding belief of an history. This will return nullptr if no such history exists
        auto corresponding_belief = this->getBeliefAt(joint_history);

        // Get the belief label (corresponding belief or inputed belief)
        auto belief_label = (corresponding_belief != nullptr) ? corresponding_belief : belief;

        // Add input probability to the current probability
        this->setProbability(joint_history, belief_label, this->getProbability(joint_history) + proba);
        OccupancyState::TIME_IN_ADD_PROBA += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);
    }

    Pair<std::shared_ptr<JointHistoryInterface>, std::shared_ptr<BeliefInterface>> OccupancyState::sampleJointHistoryBelief()
    {
        auto distribution = std::make_shared<DiscreteDistribution<std::shared_ptr<JointHistoryInterface>>>();
        for (auto const joint_history : this->getJointHistories())
        {
            distribution->setProbability(joint_history, this->getProbability(joint_history));
        }
        auto sampled_joint_history = distribution->sample();
        return std::make_pair(sampled_joint_history, this->getBeliefAt(sampled_joint_history));
    }

    bool OccupancyState::operator==(const OccupancyState &other) const
    {
        clock_t t_begin = clock();
        if (this->size() != other.size())
        {
            OccupancyState::TIME_IN_EQUAL_OPERATOR += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);
            return false;
        }

        if (std::abs(this->getDefault() - other.getDefault()) > PRECISION)
        {
            OccupancyState::TIME_IN_EQUAL_OPERATOR += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);
            return false;
        }

        // For all points in the support
        for (const auto &jhistory : this->getJointHistories())
        {   
            // if (this->getBeliefAt(jhistory)->size() != other.getBeliefAt(jhistory)->size())
            // {
            //     return false;
            // }

            // For all states in the corresponding belief
            for (const auto &state : this->getBeliefAt(jhistory)->getStates())
            {
                // Does the corresponding probabilities are equals ?
                if (std::abs(this->getProbability(jhistory, state) - other.getProbability(jhistory, state)) > OccupancyState::PRECISION)
                {
                    OccupancyState::TIME_IN_EQUAL_OPERATOR += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);
                    return false;
                }
            }
        }
        OccupancyState::TIME_IN_EQUAL_OPERATOR += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);
        return true;
    }

    bool OccupancyState::operator==(const std::shared_ptr<BeliefInterface> &other) const
    {
        return OccupancyState::operator==(*std::dynamic_pointer_cast<OccupancyState>(other->toOccupancyState()));
    }

    double OccupancyState::operator^(const std::shared_ptr<BeliefInterface> &other) const
    {
        double product = 0;

        for (const auto &jhistory : this->getJointHistories())
        {
            for (const auto &state : this->getBeliefAt(jhistory)->getStates())
            {
                product += this->getProbability(jhistory, state) * other->toOccupancyState()->getProbability(jhistory,state);
            }
        }

        return product;
    }

    // ###################################
    // ###### MANIPULATE DATA ############
    // ###################################

    const std::set<std::shared_ptr<JointHistoryInterface>> &OccupancyState::getJointHistories() const
    {
        return this->list_joint_histories_;
    }

    const std::set<std::shared_ptr<BeliefInterface>> &OccupancyState::getBeliefs() const
    {
        return this->list_beliefs_;
    }

    std::shared_ptr<BeliefInterface> OccupancyState::getBeliefAt(const std::shared_ptr<JointHistoryInterface> &jhistory) const
    {
        auto iterator_on_belief = this->map_joint_history_to_belief_.find(jhistory);
        return (iterator_on_belief == this->map_joint_history_to_belief_.end()) ? nullptr : iterator_on_belief->second;
    }

    void OccupancyState::setBeliefAt(const std::shared_ptr<JointHistoryInterface> &jhistory, const std::shared_ptr<BeliefInterface> &belief)
    {
        this->map_joint_history_to_belief_[jhistory] = belief;
    }

    const std::set<std::shared_ptr<HistoryInterface>> &OccupancyState::getIndividualHistories(number agent_id) const
    {
        return this->all_list_ihistories_[agent_id];
    }

    const std::vector<std::set<std::shared_ptr<HistoryInterface>>> &OccupancyState::getAllIndividualHistories() const
    {
        return this->all_list_ihistories_;
    }

    TypeState OccupancyState::getTypeState() const
    {
        return TypeState::OCCUPANCY_STATE;
    }

    void OccupancyState::setupIndividualHistories()
    {
        this->all_list_ihistories_.clear();
        bool first_passage = true;
        for (const auto &jhist : this->getJointHistories())
        {
            const auto &ihists = jhist->getIndividualHistories();
            for (std::size_t i = 0; i < ihists.size(); i++)
            {
                if (first_passage)
                {
                    this->all_list_ihistories_.push_back({});
                }
                this->all_list_ihistories_[i].insert(ihists[i]);
            }
            first_passage = false;
        }
    }

    void OccupancyState::setupBeliefsAndHistories()
    {
        // Get the set of joint histories that are in the support of the OccupancyState
        this->list_joint_histories_.clear();
        this->list_beliefs_.clear();
        for (const auto &joint_history_tmp : this->getStates())
        {
            auto joint_history = joint_history_tmp->toHistory()->toJointHistory();
            this->list_joint_histories_.insert(joint_history);
            this->list_beliefs_.insert(this->getBeliefAt(joint_history));
        }
    }

    void OccupancyState::setup()
    {
        this->setupBeliefsAndHistories();
        this->setupIndividualHistories();
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
        std::dynamic_pointer_cast<OccupancyState>(one_step_uncompress_ostate)->setCompressedOccupancy(this->getptr());
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
        auto iterator = this->private_ihistory_map_.at(agent_id).find(ihistory);
        return (iterator == this->private_ihistory_map_.at(agent_id).end()) ? ihistory : *iterator->second;
    }

    Joint<std::shared_ptr<HistoryInterface>> OccupancyState::getJointLabels(const Joint<std::shared_ptr<HistoryInterface>> &list_ihistories) const
    {
        Joint<std::shared_ptr<HistoryInterface>> new_list_ihistories;
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
        auto iterator = this->private_ihistory_map_[agent_id].find(ihistory);
        if (iterator != this->private_ihistory_map_[agent_id].end())
        {
            // Get the old label
            auto &&old_label_ptr = iterator->second;

            // Change every labels of ihistories that have old_label_ptr as label
            *old_label_ptr = label;
        }
        else
        {
            auto iterator_on_label_to_ptr = this->map_label_to_pointer[agent_id].find(label);
            // Check if the label is already used for another indiv history
            if (iterator_on_label_to_ptr != this->map_label_to_pointer[agent_id].end())
            {
                this->private_ihistory_map_[agent_id][ihistory] = iterator_on_label_to_ptr->second;
            }
            else
            {
                // If no such label is already used, create a pointer on it and store it
                auto &&new_ptr_on_label = std::make_shared<std::shared_ptr<HistoryInterface>>(label);
                this->map_label_to_pointer[agent_id][label] = new_ptr_on_label;
                this->private_ihistory_map_[agent_id][ihistory] = new_ptr_on_label;
            }
        }
    }

    void OccupancyState::updateJointLabels(const Joint<std::shared_ptr<HistoryInterface>> &list_ihistories, const Joint<std::shared_ptr<HistoryInterface>> &list_labels)
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
        return this->getPrivateOccupancyState(agent_identifier, ihistory_1)->check_equivalence(*this->getPrivateOccupancyState(agent_identifier, ihistory_2));
    }

    /**
     * @brief 
     * 
     * https://gitlab.inria.fr/maintenance/maintenance.html?appli=GITLAB 
     * @return std::shared_ptr<OccupancyStateInterface> 
     */
    std::shared_ptr<OccupancyStateInterface> OccupancyState::compress()
    {
        // std::cout << "Start Compress" << std::endl;
        auto current_compact_ostate = std::make_shared<OccupancyState>(this->num_agents_);
        auto previous_compact_ostate = std::make_shared<OccupancyState>(*this);

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
                            auto joint_history_from_partial = previous_compact_ostate->getPrivateOccupancyState(agent_id, ihistory_label)->getJointHistory(partial_jhist);

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

        previous_compact_ostate->setFullyUncompressedOccupancy(this->getFullyUncompressedOccupancy());
        previous_compact_ostate->setOneStepUncompressedOccupancy(this->getptr());
        // std::cout << "End Compress" << std::endl;

        return previous_compact_ostate;
    }

    void OccupancyState::setupPrivateOccupancyStates()
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

    void OccupancyState::finalize()
    {
        clock_t t_begin = clock();

        Belief::finalize();
        this->setup();
        this->setupPrivateOccupancyStates();
        this->setProbabilityOverIndividualHistories();

        OccupancyState::PASSAGE_FINALIZE++;
        OccupancyState::TIME_IN_FINALIZE += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);
    }

    void OccupancyState::normalize()
    {
        double sum = this->norm_1();
        for (const auto &joint_history : this->getJointHistories())
        {
            this->setProbability(joint_history, this->getBeliefAt(joint_history), this->getProbability(joint_history) / sum);
        }
    }

    std::shared_ptr<OccupancyState> OccupancyState::getptr()
    {
        return std::dynamic_pointer_cast<OccupancyState>(this->toState()->toOccupancyState());
    }

    std::string OccupancyState::str() const
    {
        std::ostringstream res;
        res << std::setprecision(config::OCCUPANCY_DECIMAL_PRINT) << std::fixed;

        res << "<occupancy-state defaut=\""<<this->getDefault()<<"\" \t size=\"" << MappedVector<std::shared_ptr<State>>::size() << "\">\n";
        for (const auto &history_as_state : this->getIndexes())
        {
            auto joint_history = history_as_state->toHistory()->toJointHistory();
            res << "\t<probability";
            res << " joint_history=" << joint_history->short_str() << "";
            res << " belief=" << this->getBeliefAt(joint_history)->str() << ">\n";
            res << "\t\t\t" << this->getProbability(joint_history) << "\n";
            res << "\t</probability \n";
        }
        res << "</occupancy-state>";
        return res.str();
    }

    double OccupancyState::getProbabilityOverIndividualHistories(number agent, const std::shared_ptr<HistoryInterface> &ihistory) const
    {
        return this->probability_ihistories.at(agent).at(ihistory);
    }

    void OccupancyState::setProbabilityOverIndividualHistories()
    {
        // For all agents
        for (number ag_id = 0; ag_id < this->num_agents_; ag_id++)
        {
            // For all individual history of this agent
            for (const auto &ihistory : this->getIndividualHistories(ag_id))
            {
                // Compute the probability of the individual history of agent i
                double prob = 0;
                for (const auto &pair_hidden_state_history_proba : *this->getPrivateOccupancyState(ag_id, ihistory))
                {
                    prob += pair_hidden_state_history_proba.second;
                }
                this->probability_ihistories[ag_id][ihistory] = prob;
            }
        }
    }

    // #############################################
    // ######### ACTION SPACE ######################
    // #############################################

    std::shared_ptr<Space> OccupancyState::getActionSpaceAt(number t)
    {
        if (this->action_space_map->find(t) != this->action_space_map->end())
        {
            return this->action_space_map->at(t);
        }
        else
        {
            return nullptr;
        }
    }

    void OccupancyState::setActionSpaceAt(number t, std::shared_ptr<Space> action_space)
    {
        this->action_space_map->emplace(t, action_space);
    }

    // #############################################
    // ######### ............ ######################
    // #############################################

    std::vector<std::shared_ptr<JointHistoryInterface>> OccupancyState::getIndividualHierarchicalHistoryVectorFor(number t, number agent)
    {
        return this->individual_hierarchical_history_vector_map_vector[agent]->at(t);
    }

    void OccupancyState::pushToIndividualHierarchicalHistoryVectorFor(number t, number agent, std::shared_ptr<JointHistoryInterface>& individual_hierarchical_history)
    {
        if (this->individual_hierarchical_history_vector_map_vector[agent]->find(t) == this->individual_hierarchical_history_vector_map_vector[agent]->end())
        {
            this->individual_hierarchical_history_vector_map_vector[agent]->emplace(t, std::vector<std::shared_ptr<JointHistoryInterface>>{});
        }
        this->individual_hierarchical_history_vector_map_vector[agent]->at(t).push_back(individual_hierarchical_history);
    }

} // namespace sdm
