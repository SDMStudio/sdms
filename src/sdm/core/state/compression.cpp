// #include <sdm/core/state/compression.hpp>

// namespace sdm
// {

//     // #####################################
//     // ###### MANIPULATE LABELS ############
//     // #####################################

//     std::shared_ptr<HistoryInterface> Compression::getLabel(const std::shared_ptr<HistoryInterface> &ihistory, number agent_id) const
//     {
//         auto iterator = this->maps_ihistory_to_ptr_on_label.at(agent_id).find(ihistory);
//         return (iterator == this->maps_ihistory_to_ptr_on_label.at(agent_id).end()) ? ihistory : *iterator->second;
//     }

//     Joint<std::shared_ptr<HistoryInterface>> Compression::getJointLabels(const Joint<std::shared_ptr<HistoryInterface>> &list_ihistories) const
//     {
//         Joint<std::shared_ptr<HistoryInterface>> new_list_ihistories;
//         for (int agent_id = 0; agent_id < list_ihistories->getNumAgents(); ++agent_id)
//         {
//             // if the ihistory was never compressed
//             new_list_ihistories.push_back(this->getLabel(list_ihistories.at(agent_id), agent_id));
//         }
//         return new_list_ihistories;
//     }

//     std::shared_ptr<JointHistoryInterface> Compression::getCompressedJointHistory(const std::shared_ptr<JointHistoryInterface> &joint_history) const
//     {
//         const auto &labels = this->getJointLabels(joint_history->getIndividualHistories());
//         return this->jhistory_map_.at(labels);
//     }

//     void Compression::updateLabel(number agent_id, const std::shared_ptr<HistoryInterface> &ihistory, const std::shared_ptr<HistoryInterface> &label)
//     {
//         // Check existance of a label for individual history
//         auto iterator_on_label = this->maps_ihistory_to_ptr_on_label[agent_id].find(ihistory);
//         if (iterator_on_label != this->maps_ihistory_to_ptr_on_label[agent_id].end())
//         {
//             // If a label is available
//             // Get the old label
//             auto &&old_label_ptr = iterator_on_label->second;

//             // Change every labels of ihistories that have old_label_ptr as label
//             *old_label_ptr = label;
//         }
//         else
//         {
//             // If no label exists for ihistory
//             // Check existance of the label in the list of available labels (i.e. the label is already used for another ihistory)
//             auto iterator_on_label_to_ptr = this->maps_label_to_ptr_on_label[agent_id].find(label);
//             if (iterator_on_label_to_ptr != this->maps_label_to_ptr_on_label[agent_id].end())
//             {
//                 // If such a label exists, map the ihistory to the its address
//                 this->maps_ihistory_to_ptr_on_label[agent_id][ihistory] = iterator_on_label_to_ptr->second;
//             }
//             else
//             {
//                 // If no such label is already used, create a pointer on it and store it
//                 auto &&new_ptr_on_label = std::make_shared<std::shared_ptr<HistoryInterface>>(label);
//                 // Store the address of the label in the list of labels
//                 this->maps_label_to_ptr_on_label[agent_id][label] = new_ptr_on_label;
//                 // Map the ihistory to the address of the label
//                 this->maps_ihistory_to_ptr_on_label[agent_id][ihistory] = new_ptr_on_label;
//             }
//         }
//     }

//     void Compression::updateJointLabels(const Joint<std::shared_ptr<HistoryInterface>> &list_ihistories, const Joint<std::shared_ptr<HistoryInterface>> &list_labels)
//     {
//         // For all agents
//         for (number agent_id = 0; agent_id < this->num_agents_; ++agent_id)
//         {
//             // Update individual label
//             this->updateLabel(agent_id, list_ihistories.at(agent_id), list_labels.at(agent_id));
//         }
//     }

//     // #############################################
//     // ######### MANIPULATE COMPRESSION ############
//     // #############################################

//     bool Compression::areIndividualHistoryLPE(const std::shared_ptr<HistoryInterface> &ihistory_1, const std::shared_ptr<HistoryInterface> &ihistory_2, number agent_identifier)
//     {
//         // Check probabilistic equivalence between two histories
//         return this->getPrivateOccupancyState(agent_identifier, ihistory_1)->check_equivalence(*this->getPrivateOccupancyState(agent_identifier, ihistory_2));
//     }

//     std::shared_ptr<OccupancyStateInterface> Compression::compress(const std::shared_ptr<OccupancyStateInterface> &state_to_compress)
//     {
//         number num_agents = previous_compact_ostate->getNumAgents();
//         auto previous_compact_ostate = std::make_shared<OccupancyState>(*state_to_compress);
//         auto current_compact_ostate = std::make_shared<OccupancyState>(num_agents);

//         // For all agents
//         for (number agent_id = 0; agent_id < num_agents; ++agent_id)
//         {
//             // Get all infividual histories of agent i (a set of individual histories for agent i)
//             const auto &support_set = this->getIndividualHistories(agent_id);

//             // Transform the set of ihistories to a vector in order to sort it
//             auto &&support = tools::set2vector(support_set);

//             // sort ihistories
//             std::sort(support.begin(), support.end());

//             for (auto iter_first = support.begin(); iter_first != support.end();)
//             {
//                 // Get the ihistory "label"
//                 auto ihistory_label = *iter_first;

//                 // Erase the ihistory "label" from the support
//                 iter_first = support.erase(iter_first);

//                 // Copy probabilities corresponding to the current ihistory label
//                 auto private_occupancy_state_label = previous_compact_ostate->getPrivateOccupancyState(agent_id, ihistory_one_step_left);
//                 for (const auto &joint_history : private_occupancy_state_label->getJointHistories())
//                 {
//                     for (const auto &belief : private_occupancy_state_label->getBeliefsAt(joint_history))
//                     {
//                         current_compact_ostate->setProbability(joint_history, belief, previous_compact_ostate->getProbability(joint_history, belief));
//                     }
//                 }

//                 for (auto iter_second = iter_first; iter_second != support.end();)
//                 {
//                     // Get the ihistory we want to check the probabilistic equivalence
//                     auto ihistory_one_step_left = *iter_second;
//                     // Check the equivalence
//                     if (this->areIndividualHistoryLPE(ihistory_label, ihistory_one_step_left, agent_id))
//                     {
//                         // Update the label of the ihistory
//                         this->updateLabel(agent_id, ihistory_one_step_left, ihistory_label);

//                         // Erase the unecessary equivalent individual history
//                         iter_second = support.erase(iter_second);

//                         auto private_occupancy_state_one_step_left = previous_compact_ostate->getPrivateOccupancyState(agent_id, ihistory_one_step_left);

//                         // Add probabilities corresponding to the compressed ihistory
//                         for (const auto &private_joint_history : private_occupancy_state_one_step_left->getJointHistories())
//                         {
//                             for (const auto &belief : private_occupancy_state_one_step_left->getBeliefsAt(private_joint_history))
//                             {
//                                 // Get the probability
//                                 double probability = current_compact_ostate->getProbability(private_joint_history, belief);

//                                 auto partial_jhist = private_occupancy_state_one_step_left->getPartialJointHistory(private_joint_history);
//                                 auto joint_history = private_occupancy_state_label->getJointHistory(partial_jhist);
//                                 current_compact_ostate->addProbability(joint_history, belief, probability);
//                             }
//                         }
//                     }
//                     else
//                     {
//                         iter_second++;
//                     }
//                 }
//             }

//             *previous_compact_ostate = *current_compact_ostate;
//             previous_compact_ostate->maps_ihistory_to_ptr_on_label = this->maps_ihistory_to_ptr_on_label;
//             previous_compact_ostate->finalize();
//             current_compact_ostate->clear();
//         }

//         previous_compact_ostate->setFullyUncompressedOccupancy(state_to_compress->getFullyUncompressedOccupancy());
//         previous_compact_ostate->setOneStepUncompressedOccupancy(state_to_compress);
//         return previous_compact_ostate;
//     }

//     void finalize_compression()
//     {
//         for (const auto &jhist : this->getJointHistories())
//         {
//             // Store relation between joint history and list of individual histories
//             this->jhistory_map_.emplace(jhist->getIndividualHistories(), jhist);

//             for (const auto &belief : this->getBeliefsAt(jhist))
//             {
//                 const auto &proba_history = this->getProbability(jhist, belief);

//                 // For each agent we update its private occupancy state
//                 for (number agent_id = 0; agent_id < this->num_agents_; agent_id++)
//                 {
//                     // Instanciation empty private occupancy state associated to ihistory and agent i if not exists
//                     if (this->tuple_of_maps_from_histories_to_private_occupancy_states_[agent_id].find(jhist->getIndividualHistory(agent_id)) == this->tuple_of_maps_from_histories_to_private_occupancy_states_[agent_id].end())
//                     {
//                         this->tuple_of_maps_from_histories_to_private_occupancy_states_[agent_id].emplace(jhist->getIndividualHistory(agent_id), std::make_shared<PrivateOccupancyState>(agent_id, this->num_agents_));
//                     }

//                     // Set private occupancy measure
//                     for (const auto &state : belief->getStates())
//                     {
//                         this->tuple_of_maps_from_histories_to_private_occupancy_states_[agent_id][jhist->getIndividualHistory(agent_id)]->addProbability(jhist, state, proba_history * belief->getProbability(state));
//                     }
//                     // this->tuple_of_maps_from_histories_to_private_occupancy_states_[agent_id][jhist->getIndividualHistory(agent_id)]->addProbability(jhist, belief, proba);
//                 }
//             }
//         }
//         for (number agent_id = 0; agent_id < this->num_agents_; agent_id++)
//         {
//             for (const auto &pair_ihist_private_occupancy_state : this->tuple_of_maps_from_histories_to_private_occupancy_states_[agent_id])
//             {
//                 pair_ihist_private_occupancy_state.second->finalize(false);
//             }
//         }
//     }
// }