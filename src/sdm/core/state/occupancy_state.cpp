#include <iomanip>
#include <sdm/config.hpp>
#include <sdm/exception.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>

namespace sdm
{
    double OccupancyState::PRECISION = config::PRECISION_OCCUPANCY_STATE;
    RecursiveMap<std::pair<std::shared_ptr<JointHistoryInterface>, std::shared_ptr<BeliefInterface>>, std::shared_ptr<JointHistoryBeliefPair>> OccupancyState::map_pair_to_pointer_ = {};

    OccupancyState::OccupancyState() : OccupancyState(2)
    {
    }

    OccupancyState::OccupancyState(number num_agents) : num_agents_(num_agents), Belief(num_agents), action_space_map(std::make_shared<std::unordered_map<number, std::shared_ptr<Space>>>())
    {
        for (number agent_id = 0; agent_id < num_agents; agent_id++)
        {
            this->tuple_of_maps_from_histories_to_private_occupancy_states_.push_back({});
            this->private_ihistory_map_.push_back({});
            this->map_label_to_pointer.push_back({});
        }
    }

    OccupancyState::OccupancyState(const OccupancyState &occupancy_state)
        : Belief(occupancy_state),
          num_agents_(occupancy_state.num_agents_),
          tuple_of_maps_from_histories_to_private_occupancy_states_(occupancy_state.tuple_of_maps_from_histories_to_private_occupancy_states_),
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
          probability_jhistories(occupancy_state.probability_jhistories),
          action_space_map(std::make_shared<std::unordered_map<number, std::shared_ptr<Space>>>())
    {
    }

    bool OccupancyState::operator==(const OccupancyState &other) const
    {
        bool are_equal = MappedVector<std::shared_ptr<State>, double>::is_equal(other, PRECISION);
        return are_equal;
    }

    double OccupancyState::getProbability(const std::shared_ptr<State> &pair_history_belief) const
    {
        return Belief::getProbability(pair_history_belief);
    }

    double OccupancyState::getProbability(const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<BeliefInterface> &belief) const
    {
        auto iterator_on_pair_history_belief = this->map_pair_to_pointer_.find({joint_history, belief});
        return (iterator_on_pair_history_belief == this->map_pair_to_pointer_.end()) ? this->getDefaultValue() : this->getProbability(iterator_on_pair_history_belief->second);
    }

    void OccupancyState::setProbability(const std::shared_ptr<State> &pair_history_belief, double proba)
    {
        const auto &casted_pair = std::static_pointer_cast<JointHistoryBeliefPair>(pair_history_belief);
        this->map_pair_to_pointer_[{casted_pair->first, casted_pair->second}] = casted_pair;
        return Belief::setProbability(pair_history_belief, proba);
    }

    void OccupancyState::setProbability(const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<BeliefInterface> &belief, double proba)
    {
        auto iterator_on_pair_history_belief = this->map_pair_to_pointer_.find({joint_history, belief});
        if (iterator_on_pair_history_belief == this->map_pair_to_pointer_.end())
        {
            auto pair_hist_belief = std::make_shared<JointHistoryBeliefPair>(joint_history, belief);
            this->map_pair_to_pointer_[{joint_history, belief}] = pair_hist_belief;
            return Belief::setProbability(pair_hist_belief, proba);
        }
        else
        {
            return Belief::setProbability(this->getPairPointer(joint_history, belief), proba);
        }
    }

    void OccupancyState::addProbability(const std::shared_ptr<State> &pair_history_belief, double proba)
    {
        const auto &casted_pair = std::static_pointer_cast<JointHistoryBeliefPair>(pair_history_belief);
        this->map_pair_to_pointer_[{casted_pair->first, casted_pair->second}] = casted_pair;
        return Belief::addProbability(pair_history_belief, proba);
    }

    void OccupancyState::addProbability(const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<BeliefInterface> &belief, double proba)
    {
        auto iterator_on_pair_history_belief = this->map_pair_to_pointer_.find({joint_history, belief});
        if (iterator_on_pair_history_belief == this->map_pair_to_pointer_.end())
        {
            auto pair_hist_belief = std::make_shared<JointHistoryBeliefPair>(joint_history, belief);
            this->map_pair_to_pointer_[{joint_history, belief}] = pair_hist_belief;
            return Belief::addProbability(pair_hist_belief, proba);
        }
        else
        {
            return Belief::addProbability(this->getPairPointer(joint_history, belief), proba);
        }
    }

    Pair<std::shared_ptr<JointHistoryInterface>, std::shared_ptr<BeliefInterface>> OccupancyState::sampleJointHistoryBelief()
    {
        auto distribution = std::make_shared<DiscreteDistribution<Pair<std::shared_ptr<JointHistoryInterface>, std::shared_ptr<BeliefInterface>>>>();
        for (auto const joint_history: this->getJointHistories())
        {
            for (auto const belief: this->getBeliefsAt(joint_history))
            {
                distribution->setProbability(std::make_pair(joint_history, belief), this->getProbability(joint_history, belief));
            }
        }
        return distribution->sample();
    }

    bool OccupancyState::operator==(const std::shared_ptr<BeliefInterface> &other) const
    {
        if (this->size() != other->size())
        {
            return false;
        }

        for (const auto &jhistory : this->getJointHistories())
        {
            for (const auto &belief : this->getBeliefsAt(jhistory))
            {
                if (this->getProbability(jhistory, belief) != other->toOccupancyState()->getProbability(jhistory, belief))
                {
                    return false;
                }
            }
        }

        return true;
    }

    double OccupancyState::operator^(const std::shared_ptr<BeliefInterface> &other) const
    {
        double product = 0;

        // std::shared_ptr<OccupancyStateInterface> occupancy_bigger;
        // std::shared_ptr<OccupancyStateInterface> occupancy_smaller;

        // if(this->size() < other->size())
        // {
        //     occupancy_smaller = this->getPointer()->toState()->toOccupancyState();
        //     occupancy_bigger = other->toOccupancyState();
        // }else
        // {
        //     occupancy_bigger = this->getPointer()->toState()->toOccupancyState();
        //     occupancy_smaller = other->toOccupancyState();
        // }

        for (const auto &jhistory : this->getJointHistories())
        {
            for (const auto &belief : this->getBeliefsAt(jhistory))
            {
                product += this->getProbability(jhistory, belief) * other->toOccupancyState()->getProbability(jhistory, belief);
            }
        }

        return product;
    }

    std::shared_ptr<JointHistoryBeliefPair> OccupancyState::getPairPointer(const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<BeliefInterface> &belief) const
    {
        return this->map_pair_to_pointer_.at({joint_history, belief});
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

    const std::set<std::shared_ptr<BeliefInterface>> &OccupancyState::getBeliefsAt(const std::shared_ptr<JointHistoryInterface> &jhistory) const
    {
        return this->map_joint_history_to_belief_.at(jhistory);
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
        for (const auto &pair_history_belief : this->getStates())
        {
            auto history = std::static_pointer_cast<JointHistoryBeliefPair>(pair_history_belief)->first;
            auto belief = std::static_pointer_cast<JointHistoryBeliefPair>(pair_history_belief)->second;
            this->list_joint_histories_.insert(history);
            this->list_beliefs_.insert(belief);
            this->map_joint_history_to_belief_[history].insert(belief);
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

    std::shared_ptr<OccupancyStateInterface> OccupancyState::compress()
    {
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
                    for (const auto &belief : previous_compact_ostate->getPrivateOccupancyState(agent_id, ihistory_label)->getBeliefsAt(joint_history))
                    {
                        current_compact_ostate->setProbability(joint_history, belief, previous_compact_ostate->getProbability(joint_history, belief));
                    }
                }

                for (auto iter_second = iter_first; iter_second != support.end();)
                {
                    auto ihistory_one_step_left = *iter_second; // Get the ihistory we want check the equivalence
                    if (this->areIndividualHistoryLPE(ihistory_label, ihistory_one_step_left, agent_id))
                    {
                        // Store new label
                        this->updateLabel(agent_id, ihistory_one_step_left, ihistory_label);

                        // Erase uncessary equivalent individual history
                        iter_second = support.erase(iter_second);
                        for (const auto &pair_history_belief : previous_compact_ostate->getPrivateOccupancyState(agent_id, ihistory_one_step_left)->getStates())
                        {
                            auto private_joint_history = std::static_pointer_cast<JointHistoryBeliefPair>(pair_history_belief)->first;
                            auto belief = std::static_pointer_cast<JointHistoryBeliefPair>(pair_history_belief)->second;
                            double probability = previous_compact_ostate->getPrivateOccupancyState(agent_id, ihistory_one_step_left)->getProbability(pair_history_belief);

                            auto partial_jhist = previous_compact_ostate->getPrivateOccupancyState(agent_id, ihistory_one_step_left)->getPartialJointHistory(private_joint_history);
                            auto joint_history = previous_compact_ostate->getPrivateOccupancyState(agent_id, ihistory_label)->getJointHistory(partial_jhist);
                            current_compact_ostate->addProbability(joint_history, belief, probability);
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
        return previous_compact_ostate;
    }

    void OccupancyState::finalize()
    {
        this->setup();

        for (const auto &jhist : this->getJointHistories())
        {
            for (const auto &belief : this->getBeliefsAt(jhist))
            {
                const auto &proba = this->getProbability(jhist, belief);

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
        }
        for (number agent_id = 0; agent_id < this->num_agents_; agent_id++)
        {
            for (const auto &pair_ihist_private_occupancy_state : this->tuple_of_maps_from_histories_to_private_occupancy_states_[agent_id])
            {
                pair_ihist_private_occupancy_state.second->finalize(false);
            }
        }
        this->setProbabilityOverJointHistory();
        this->setProbabilityOverIndividualHistories();
    }

    std::shared_ptr<OccupancyState> OccupancyState::getptr()
    {
        return std::static_pointer_cast<OccupancyState>(this->toState()->toOccupancyState());
    }

    std::string OccupancyState::str() const
    {
        std::ostringstream res;
        res << std::setprecision(config::OCCUPANCY_DECIMAL_PRINT) << std::fixed;

        res << "<occupancy-state size=\"" << MappedVector<std::shared_ptr<State>>::size() << "\">\n";
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
        res << "</occupancy-state>";
        return res.str();
    }

    const double &OccupancyState::getProbabilityOverIndividualHistories(number agent, const std::shared_ptr<HistoryInterface> &ihistory) const
    {
        return this->probability_ihistories.at(agent).at(ihistory);
    }

    const double &OccupancyState::getProbabilityOverJointHistory(const std::shared_ptr<JointHistoryInterface> &joint_history) const
    {
        return this->probability_jhistories.at(joint_history);
    }

    void OccupancyState::setProbabilityOverJointHistory()
    {
        for (const auto &joint_history : this->getJointHistories())
        {
            for (const auto &belief : this->getBeliefsAt(joint_history))
            {
                this->probability_jhistories[joint_history] += this->getProbability(joint_history, belief);
            }
        }
    }

    void OccupancyState::setProbabilityOverIndividualHistories()
    {
        for (number ag_id = 0; ag_id < this->num_agents_; ag_id++)
        {
            for (const auto &ihistory : this->getIndividualHistories(ag_id))
            {
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

} // namespace sdm
