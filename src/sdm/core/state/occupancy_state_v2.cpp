#include <sdm/core/state/occupancy_state_v2.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>
#include <sdm/exception.hpp>

namespace sdm
{
    double OccupancyState::PRECISION = config::PRECISION_OCCUPANCY_STATE;

    OccupancyState::OccupancyState(double default_value) : OccupancyState(2, default_value)
    {
    }

    OccupancyState::OccupancyState(number num_agents, double default_value) : MappedVector<std::shared_ptr<BaseState<Pair<std::shared_ptr<State>,std::shared_ptr<JointHistoryTreeInterface>>>>, double>(default_value), num_agents_(num_agents)
    {
        for (number agent_id = 0; agent_id < num_agents; agent_id++)
        {
            this->tuple_of_maps_from_histories_to_private_occupancy_states_.push_back({});
            this->private_ihistory_map_.push_back({});
            this->map_label_to_pointer.push_back({});
        }
    }

    OccupancyState::OccupancyState(const OccupancyState &occupancy_state)
        : MappedVector<std::shared_ptr<BaseState<Pair<std::shared_ptr<State>,std::shared_ptr<JointHistoryTreeInterface>>>>, double>(occupancy_state),
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

    const Joint<RecursiveMap<std::shared_ptr<HistoryTreeInterface>, std::shared_ptr<PrivateOccupancyState>>> &OccupancyState::getPrivateOccupancyStates() const
    {
        return this->tuple_of_maps_from_histories_to_private_occupancy_states_;
    }

    const std::shared_ptr<PrivateOccupancyState> &OccupancyState::getPrivateOccupancyState(const number &agent_id, const std::shared_ptr<HistoryTreeInterface> &ihistory) const
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

    std::shared_ptr<HistoryTreeInterface> OccupancyState::getLabel(const std::shared_ptr<HistoryTreeInterface> &ihistory, number agent_id) const
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

    std::vector<std::shared_ptr<HistoryTreeInterface>> OccupancyState::getJointLabels(const std::vector<std::shared_ptr<HistoryTreeInterface>> &list_ihistories) const
    {
        std::vector<std::shared_ptr<HistoryTreeInterface>> new_list_ihistories;
        for (int agent_id = 0; agent_id < this->num_agents_; ++agent_id)
        {
            // if the ihistory was never compressed
            new_list_ihistories.push_back(this->getLabel(list_ihistories.at(agent_id), agent_id));
        }
        return new_list_ihistories;
    }

    void OccupancyState::updateLabel(number agent_id, const std::shared_ptr<HistoryTreeInterface> &ihistory, const std::shared_ptr<HistoryTreeInterface> &label)
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

    void OccupancyState::updateJointLabels(const std::vector<std::shared_ptr<HistoryTreeInterface>> &list_ihistories, const std::vector<std::shared_ptr<HistoryTreeInterface>> &list_labels)
    {
        for (number agent_id = 0; agent_id < this->num_agents_; ++agent_id)
        {
            this->updateLabel(agent_id, list_ihistories.at(agent_id), list_labels.at(agent_id));
        }
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

    std::shared_ptr<JointHistoryTreeInterface> OccupancyState::getCompressedJointHistory(const std::shared_ptr<JointHistoryTreeInterface> &joint_history) const
    {
        const auto &labels = this->getJointLabels(joint_history->getIndividualHistories());
        return this->jhistory_map_.at(labels);
    }

    bool OccupancyState::areIndividualHistoryLPE(const std::shared_ptr<HistoryTreeInterface> &ihistory_1, const std::shared_ptr<HistoryTreeInterface> &ihistory_2, number agent_identifier)
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
            for (const auto& state : this->getStatesAt(jhist))
            {
                const auto &proba = this->getProbability(state,jhist);

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
                    this->tuple_of_maps_from_histories_to_private_occupancy_states_[agent_id][jhist->getIndividualHistory(agent_id)]->addProbability(state,jhist, proba);
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

    const double &OccupancyState::getProbabilityOverIndividualHistories(number agent, const std::shared_ptr<HistoryTreeInterface> &ihistory) const
    {
        return this->probability_ihistories.at(agent).at(ihistory);
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


    // *********************************************************
    // *********************************************************
    // *********************************************************
    // *********************************************************
    // *********************************************************
    // *********************************************************
    // *********************************************************
    // *********************************************************
    // *********************************************************

    void OccupancyState::setProbability(const std::shared_ptr<State> &state, double proba)
    {
        // Set the new occupancy measure
        auto pair_state_hist = std::static_pointer_cast<BaseState<Pair<std::shared_ptr<State>, std::shared_ptr<JointHistoryTreeInterface>>>>(state);
        (*this)[pair_state_hist] = proba;
    }

    void OccupancyState::setProbability(const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryTreeInterface> &jhist, double proba)
    {
        this->setProbability(this->HiddenStateAndJointHistoryToState(state, jhist), proba);
    }

    void OccupancyState::addProbability(const std::shared_ptr<State> &state, double proba)
    {
        auto pair_state_hist = std::static_pointer_cast<BaseState<Pair<std::shared_ptr<State>, std::shared_ptr<JointHistoryTreeInterface>>>>(state);
        if (this->find(pair_state_hist) != this->end())
        {
            (*this)[pair_state_hist] += proba;
        }
        else
        {
            this->setProbability(pair_state_hist, proba);
        }
    }

    void OccupancyState::addProbability(const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryTreeInterface> &jhist, double proba)
    {
        this->addProbability(this->HiddenStateAndJointHistoryToState(state, jhist), proba);
    }

    double OccupancyState::getProbability(const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryTreeInterface> &jhist) const
    {
        // Set the new occupancy measure
        return this->getProbability(this->HiddenStateAndJointHistoryToState(state, jhist));
    }

    double OccupancyState::getProbability(const std::shared_ptr<State> &state) const
    {
        // Set the new occupancy measure
        auto pair_state_hist = std::static_pointer_cast<BaseState<Pair<std::shared_ptr<State>, std::shared_ptr<JointHistoryTreeInterface>>>>(state);

        return this->at(pair_state_hist);
    }

    std::shared_ptr<State> OccupancyState::HiddenStateAndJointHistoryToState(const std::shared_ptr<State>&state, const std::shared_ptr<JointHistoryTreeInterface>&jhist) const
    {
        return std::make_shared<BaseState<Pair<std::shared_ptr<State>, std::shared_ptr<JointHistoryTreeInterface>>>>(std::make_pair(state, jhist));
    }


    const std::vector<std::set<std::shared_ptr<HistoryTreeInterface>>> &OccupancyState::getAllIndividualHistories() const
    {
        return this->all_list_ihistories;
    }

    void OccupancyState::setAllIndividualHistories()
    {
        this->all_list_ihistories.clear();
        bool first_passage = true;
        for (const auto &jhist : this->getJointHistories())
        {
            const auto &ihists = jhist->getIndividualHistories();
            for (std::size_t i = 0; i < ihists.size(); i++)
            {
                if (first_passage)
                {
                    this->all_list_ihistories.push_back({});
                }

                this->all_list_ihistories[i].insert(ihists[i]);
            }
            first_passage = false;
        }
    }

    const std::set<std::shared_ptr<JointHistoryTreeInterface>> &OccupancyState::getJointHistories() const
    {
        // Get the set of joint histories that are in the support of the OccupancyState
        return this->list_jhistories;
    }

    void OccupancyState::setJointHistories()
    {
        // Get the set of joint histories that are in the support of the OccupancyState
        this->list_jhistories.clear();
        for (const auto &key : *this)
        {
            this->list_jhistories.insert(this->getHistory(key.first));
        }
    }

    const std::set<std::shared_ptr<State>> &OccupancyState::getStatesAt(const std::shared_ptr<JointHistoryTreeInterface> &jhistory) const
    {
        return this->list_jhistory_states.at(jhistory);
    }

    std::vector<std::shared_ptr<State>> OccupancyState::getStates() const
    {
        return this->list_states;
    }

    void OccupancyState::setStates()
    {
        for (const auto &jhist : this->getJointHistories())
        {
            for(const auto &state : this->getStatesAt(jhist))
            {
                this->list_states.push_back(state);
                this->list_jhistory_states[jhist].insert(state);
            }
        }
    }

    const std::set<std::shared_ptr<HistoryTreeInterface>> &OccupancyState::getIndividualHistories(number ag_id) const
    {
        return this->all_list_ihistories[ag_id];
    }

    std::shared_ptr<State> OccupancyState::getHiddenState(const std::shared_ptr<BaseState<Pair<std::shared_ptr<State>, std::shared_ptr<JointHistoryTreeInterface>>>> &pair_state_hist) const
    {
        return pair_state_hist->getState().first;
    }

    std::shared_ptr<JointHistoryTreeInterface> OccupancyState::getHistory(const std::shared_ptr<BaseState<Pair<std::shared_ptr<State>, std::shared_ptr<JointHistoryTreeInterface>>>> &pair_state_hist) const
    {
        return pair_state_hist->getState().second;
    }

    const std::set<std::shared_ptr<JointHistoryTreeInterface>> &OccupancyState::getJointHistoryOverIndividualHistories(number agent_id, const std::shared_ptr<HistoryTreeInterface> &indiv_history) const
    {
        return this->ihistories_to_jhistory.at(agent_id).at(indiv_history);
    }

    void OccupancyState::setJointHistoryOverIndividualHistories()
    {
        for (number ag_id = 0; ag_id < this->num_agents_; ag_id++)
        {
            this->ihistories_to_jhistory.emplace(ag_id, std::unordered_map<std::shared_ptr<HistoryTreeInterface>, std::set<std::shared_ptr<JointHistoryTreeInterface>>>());

            for (const auto &ihistory : this->getIndividualHistories(ag_id))
            {
                this->ihistories_to_jhistory.at(ag_id).emplace(ihistory, std::set<std::shared_ptr<JointHistoryTreeInterface>>());
                for (const auto &joint_history : this->getJointHistories())
                {
                    if (joint_history->getIndividualHistory(ag_id) == ihistory)
                    {
                        this->ihistories_to_jhistory.at(ag_id).at(ihistory).insert(joint_history);
                    }
                }
            }
        }
    }

    const double &OccupancyState::getProbabilityOverJointHistory(const std::shared_ptr<JointHistoryTreeInterface>&joint_history) const
    {
        return this->probability_jhistories.at(joint_history);
    }

    void OccupancyState::setProbabilityOverJointHistory()
    {
        for (const auto &state : *this)
        {
            this->probability_jhistories[this->getHistory(state.first)] += state.second;
        }
    }

    TypeState OccupancyState::getTypeState() const
    {
        return TypeState::OccupancyState_;
    }

    size_t OccupancyState::size() const
    {
        return MappedVector<std::shared_ptr<BaseState<Pair<std::shared_ptr<State>,std::shared_ptr<JointHistoryTreeInterface>>>>>::size();
    }

    void OccupancyState::setDefaultValue(double default_value)
    {
        this->setDefault(default_value);
    }
    double OccupancyState::getDefaultValue() const
    {
        this->getDefault();
    }

    std::string OccupancyState::str() const
    {

        // std::ostringstream res, tmp;
        // number horizon;
        // std::unordered_map<std::shared_ptr<JointHistoryTreeInterface>, std::pair<double, MappedVector<std::shared_ptr<State>, double>>> map;
        // for (const auto &pair_x_o_p : *this)
        // {
        //     if (map.find(pair_x_o_p.first.second) == map.end())
        //     {
        //         map.emplace(pair_x_o_p.first.second, std::make_pair(0, MappedVector<std::shared_ptr<State>, double>()));
        //         horizon = pair_x_o_p.first.second->getDepth();
        //     }
        //     map[pair_x_o_p.first.second].first += pair_x_o_p.second;
        //     map[pair_x_o_p.first.second].second[pair_x_o_p.first.first] = pair_x_o_p.second;
        // }

        // for (const auto &pair_x_o_p : *this)
        // {
        //     map[pair_x_o_p.first.second].second[pair_x_o_p.first.first] /= map[pair_x_o_p.first.second].first;
        // }

        // res << "<occupancy-state size=\"" << map.size() << "\" horizon=\"" << horizon << "\">" << std::endl;
        // for (const auto pair_o_pair_proba_belief : map)
        // {
        //     auto joint_hist = pair_o_pair_proba_belief.first;
        //     res << "\t<joint-history value=\"" << joint_hist->short_str() << "\" proba=" << pair_o_pair_proba_belief.second.first << " belief=" << pair_o_pair_proba_belief.second.second << "/>" << std::endl;
        // }
        // res << "</occupancy-state>" << std::endl;

        // return res.str();
    }

    // template <>
    // std::string OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>::str() const
    // {
    //     std::ostringstream res, tmp;
    //     res << "<occupancy-state size=\"" << this->size() << "\" horizon=\"" << "\">" << std::endl;
    //     for (const auto &pair_belief_history_proba : *this)
    //     {
    //         auto joint_hist = pair_belief_history_proba.first.second;
    //         res << "\t<joint-history value=\"" << joint_hist->short_str() << "\" proba=" << pair_belief_history_proba.second << " belief=" << pair_belief_history_proba.first.first->getData() << "/>" << std::endl;
    //     }
    //     res << "</occupancy-state>" << std::endl;

    //     return res.str();
    // }

    std::string OccupancyState::str_hyperplan() const
    {

        // std::ostringstream res, tmp;
        // number horizon;

        // std::unordered_set<TJointHistory_p> set;
        // for (const auto &pair_x_o_p : *this)
        // {
        //     set.emplace(pair_x_o_p.first.second);
        //     horizon = pair_x_o_p.first.second->getDepth();
        // }

        // res << "<simulataneous-hyperplan size=\"" << set.size() << "\" horizon=\"" << horizon << "\">" << std::endl;
        // for (const auto joint_hist : set)
        // {
        //     res << "\t<joint-history name=\"" << joint_hist->short_str() << "\" vector =" << MappedVector<Pair, double>::str() << "/>" << std::endl;
        // }
        // res << "</simulataneous-hyperplan>" << std::endl;

        // return res.str();
    }

    bool OccupancyState::operator==(const std::shared_ptr<BeliefInterface> &) const
    {
        // auto state = std::static_pointer_cast<OccupancyState>(other->toOccupancyState());
        // return MappedVector<Pair<std::shared_ptr<State>, std::shared_ptr<JointHistoryTreeInterface>>, double>::is_equal(state, PRECISION);
        throw sdm::exception::NotImplementedException();

    }

    double OccupancyState::operator^(const std::shared_ptr<BeliefInterface> &) const
    {
        throw sdm::exception::NotImplementedException();
    }

    double OccupancyState::norm_1() const
    {
        return MappedVector<std::shared_ptr<BaseState<Pair<std::shared_ptr<State>,std::shared_ptr<JointHistoryTreeInterface>>>>>::norm_1();
    }


    const std::shared_ptr<BeliefInterface> OccupancyState::createBelief(const std::shared_ptr<JointHistoryTreeInterface> &joint_history) const
    {
        std::shared_ptr<BeliefInterface> belief;

        //Go over all hidden state conditionning to a joint history
        for (auto hidden_state : this->getStatesAt(joint_history))
        {
            belief->addProbability(hidden_state,this->getProbability(this->HiddenStateAndJointHistoryToState(hidden_state, joint_history)));
        }
        return belief;
    }

    const std::shared_ptr<BeliefInterface> OccupancyState::createBeliefWeighted(const std::shared_ptr<JointHistoryTreeInterface> &joint_history) const
    {
        auto belief = this->createBelief(joint_history);

        double sum = belief->norm_1();
        for (const auto &state : belief->getStates())
        {
            belief->setProbability(state,belief->getProbability(state) / sum);
        }
        return belief;
    }


} // namespace sdm

namespace std
{
    template <>
    struct hash<sdm::OccupancyState>
    {
        typedef sdm::OccupancyState argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type &in) const
        {
            // return std::hash<sdm::MappedVector()(in);
        }
    };
}