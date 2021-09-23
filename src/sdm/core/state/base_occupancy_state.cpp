// #include <iomanip>
// #include <sdm/config.hpp>
// #include <sdm/exception.hpp>
// #include <sdm/core/state/occupancy_state.hpp>
// #include <sdm/core/state/private_occupancy_state.hpp>

// namespace sdm
// {
//     double BaseOccupancyState::PRECISION = config::PRECISION_OCCUPANCY_STATE;
//     RecursiveMap<std::pair<std::shared_ptr<JointHistoryInterface>, std::shared_ptr<BeliefInterface>>, std::shared_ptr<JointHistoryBeliefPair>> BaseOccupancyState::map_pair_hist_belief_to_pointer_ = {};
//     RecursiveMap<std::pair<std::shared_ptr<JointHistoryInterface>, std::shared_ptr<State>>, std::shared_ptr<JointHistoryStatePair>> BaseOccupancyState::map_pair_hist_state_to_pointer_ = {};

//     BaseOccupancyState::BaseOccupancyState() : OccupancyState(2)
//     {
//     }

//     BaseOccupancyState::BaseOccupancyState(number num_agents) : num_agents_(num_agents), Belief(num_agents), action_space_map(std::make_shared<std::unordered_map<number, std::shared_ptr<Space>>>())
//     {
//         for (number agent_id = 0; agent_id < num_agents; agent_id++)
//         {
//             this->tuple_of_maps_from_histories_to_private_occupancy_states_.push_back({});
//             this->private_ihistory_map_.push_back({});
//             this->map_label_to_pointer.push_back({});
//         }
//     }

//     BaseOccupancyState::BaseOccupancyState(const BaseOccupancyState &occupancy_state)
//         : Belief(occupancy_state),
//           num_agents_(occupancy_state.num_agents_),
//           jhistory_map_(occupancy_state.jhistory_map_),
//           probability_ihistories(occupancy_state.probability_ihistories),
//           list_beliefs_(occupancy_state.list_beliefs_),
//           list_joint_histories_(occupancy_state.list_joint_histories_),
//           all_list_ihistories_(occupancy_state.all_list_ihistories_),
//           map_joint_history_to_belief_(occupancy_state.map_joint_history_to_belief_),
//           ihistories_to_jhistory_(occupancy_state.ihistories_to_jhistory_),
//           probability_jhistories(occupancy_state.probability_jhistories),
//           action_space_map(std::make_shared<std::unordered_map<number, std::shared_ptr<Space>>>())
//     {
//     }

//     bool BaseOccupancyState::operator==(const OccupancyState &other) const
//     {
//         bool are_equal = MappedVector<std::shared_ptr<State>, double>::is_equal(other, PRECISION);
//         return are_equal;
//     }

//     double BaseOccupancyState::getProbability(const std::shared_ptr<State> &pair_history_belief) const
//     {
//         return Belief::getProbability(pair_history_belief);
//     }

//     double BaseOccupancyState::getProbability(const std::shared_ptr<State> &joint_history, const std::shared_ptr<State> &belief) const
//     {
//         auto iterator_on_pair_history_belief = this->map_pair_hist_belief_to_pointer_.find({joint_history->toHistory()->toJointHistory(), belief->toBelief()});
//         return (iterator_on_pair_history_belief == this->map_pair_hist_belief_to_pointer_.end()) ? this->getDefaultValue() : this->getProbability(iterator_on_pair_history_belief->second);
//     }

//     void BaseOccupancyState::setProbability(const std::shared_ptr<State> &pair_history_belief, double proba)
//     {
//         const auto &casted_pair = std::static_pointer_cast<JointHistoryBeliefPair>(pair_history_belief);
//         this->map_pair_hist_belief_to_pointer_[{casted_pair->first, casted_pair->second}] = casted_pair;
//         return Belief::setProbability(pair_history_belief, proba);
//     }

//     void BaseOccupancyState::setProbability(const std::shared_ptr<State> &joint_history, const std::shared_ptr<State> &belief, double proba)
//     {
//         auto iterator_on_pair_history_belief = this->map_pair_hist_belief_to_pointer_.find({joint_history->toHistory()->toJointHistory(), belief->toBelief()});
//         if (iterator_on_pair_history_belief == this->map_pair_hist_belief_to_pointer_.end())
//         {
//             auto pair_hist_belief = std::make_shared<JointHistoryBeliefPair>(joint_history->toHistory()->toJointHistory(), belief->toBelief());
//             this->map_pair_hist_belief_to_pointer_[{joint_history->toHistory()->toJointHistory(), belief->toBelief()}] = pair_hist_belief;
//             return Belief::setProbability(pair_hist_belief, proba);
//         }
//         else
//         {
//             return Belief::setProbability(this->getPairPointer(joint_history->toHistory()->toJointHistory(), belief->toBelief()), proba);
//         }
//     }

//     void BaseOccupancyState::addProbability(const std::shared_ptr<State> &pair_history_belief, double proba)
//     {
//         const auto &casted_pair = std::static_pointer_cast<JointHistoryBeliefPair>(pair_history_belief);
//         this->map_pair_hist_belief_to_pointer_[{casted_pair->first, casted_pair->second}] = casted_pair;
//         return Belief::addProbability(pair_history_belief, proba);
//     }

//     void BaseOccupancyState::addProbability(const std::shared_ptr<State> &joint_history, const std::shared_ptr<State> &belief, double proba)
//     {
//         auto iterator_on_pair_history_belief = this->map_pair_hist_belief_to_pointer_.find({joint_history->toHistory()->toJointHistory(), belief->toBelief()});
//         if (iterator_on_pair_history_belief == this->map_pair_hist_belief_to_pointer_.end())
//         {
//             auto pair_hist_belief = std::make_shared<JointHistoryBeliefPair>(joint_history->toHistory()->toJointHistory(), belief->toBelief());
//             this->map_pair_hist_belief_to_pointer_[{joint_history->toHistory()->toJointHistory(), belief->toBelief()}] = pair_hist_belief;
//             return Belief::addProbability(pair_hist_belief, proba);
//         }
//         else
//         {
//             return Belief::addProbability(this->getPairPointer(joint_history->toHistory()->toJointHistory(), belief), proba);
//         }
//     }

//     bool BaseOccupancyState::operator==(const std::shared_ptr<BeliefInterface> &other) const
//     {
//         if (this->size() != other->size())
//         {
//             return false;
//         }

//         for (const auto &jhistory : this->getJointHistories())
//         {
//             for (const auto &belief : this->getBeliefsAt(jhistory))
//             {
//                 if (this->getProbability(jhistory, belief) != other->toOccupancyState()->getProbability(jhistory, belief))
//                 {
//                     return false;
//                 }
//             }
//         }

//         return true;
//     }

//     double BaseOccupancyState::operator^(const std::shared_ptr<BeliefInterface> &other) const
//     {
//         double product = 0;

//         // std::shared_ptr<OccupancyStateInterface> occupancy_bigger;
//         // std::shared_ptr<OccupancyStateInterface> occupancy_smaller;

//         // if(this->size() < other->size())
//         // {
//         //     occupancy_smaller = this->getPointer()->toState()->toOccupancyState();
//         //     occupancy_bigger = other->toOccupancyState();
//         // }else
//         // {
//         //     occupancy_bigger = this->getPointer()->toState()->toOccupancyState();
//         //     occupancy_smaller = other->toOccupancyState();
//         // }

//         for (const auto &jhistory : this->getJointHistories())
//         {
//             for (const auto &belief : this->getBeliefsAt(jhistory))
//             {
//                 product += this->getProbability(jhistory, belief) * other->toOccupancyState()->getProbability(jhistory, belief);
//             }
//         }

//         return product;
//     }

//     std::shared_ptr<JointHistoryBeliefPair> BaseOccupancyState::getPairPointer(const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<State> &belief) const
//     {
//         return this->map_pair_hist_belief_to_pointer_.at({joint_history->toHistory()->toJointHistory(), belief->toBelief()});
//     }

//     std::shared_ptr<JointHistoryStatePair> BaseOccupancyState::getPairHistStatePointer(const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<State> &belief) const
//     {
//         return this->map_pair_hist_state_to_pointer_.at({joint_history->toHistory()->toJointHistory(), belief});
//     }

//     // ###################################
//     // ###### MANIPULATE DATA ############
//     // ###################################

//     const std::set<std::shared_ptr<JointHistoryInterface>> &BaseOccupancyState::getJointHistories() const
//     {
//         return this->list_joint_histories_;
//     }

//     const std::set<std::shared_ptr<BeliefInterface>> &BaseOccupancyState::getBeliefs() const
//     {
//         return this->list_beliefs_;
//     }

//     const std::set<std::shared_ptr<BeliefInterface>> &BaseOccupancyState::getBeliefsAt(const std::shared_ptr<JointHistoryInterface> &jhistory) const
//     {
//         return this->map_joint_history_to_belief_.at(jhistory);
//     }

//     const std::set<std::shared_ptr<HistoryInterface>> &BaseOccupancyState::getIndividualHistories(number agent_id) const
//     {
//         return this->all_list_ihistories_[agent_id];
//     }

//     const std::vector<std::set<std::shared_ptr<HistoryInterface>>> &BaseOccupancyState::getAllIndividualHistories() const
//     {
//         return this->all_list_ihistories_;
//     }

//     const double &BaseOccupancyState::getProbabilityOverIndividualHistories(number agent, const std::shared_ptr<HistoryInterface> &ihistory) const
//     {
//         return this->probability_ihistories.at(agent).at(ihistory);
//     }

//     const double &BaseOccupancyState::getProbabilityOverJointHistory(const std::shared_ptr<JointHistoryInterface> &joint_history) const
//     {
//         return this->probability_jhistories.at(joint_history);
//     }

//     void BaseOccupancyState::setupIndividualHistories()
//     {
//         this->all_list_ihistories_.clear();
//         bool first_passage = true;
//         for (const auto &jhist : this->getJointHistories())
//         {
//             const auto &ihists = jhist->getIndividualHistories();
//             for (std::size_t i = 0; i < ihists.size(); i++)
//             {
//                 if (first_passage)
//                 {
//                     this->all_list_ihistories_.push_back({});
//                 }

//                 this->all_list_ihistories_[i].insert(ihists[i]);
//             }
//             first_passage = false;
//         }
//     }

//     void BaseOccupancyState::setupBeliefsAndHistories()
//     {
//         // Get the set of joint histories that are in the support of the OccupancyState
//         this->list_joint_histories_.clear();
//         for (const auto &pair_history_belief : this->getStates())
//         {
//             auto history = std::static_pointer_cast<JointHistoryBeliefPair>(pair_history_belief)->first;
//             auto belief = std::static_pointer_cast<JointHistoryBeliefPair>(pair_history_belief)->second;
//             this->list_joint_histories_.insert(history);
//             this->list_beliefs_.insert(belief);
//             this->map_joint_history_to_belief_[history].insert(belief);
//         }
//     }

//     void BaseOccupancyState::setProbabilityOverJointHistory()
//     {
//         for (const auto &joint_history : this->getJointHistories())
//         {
//             for (const auto &belief : this->getBeliefsAt(joint_history))
//             {
//                 this->probability_jhistories[joint_history] += this->getProbability(joint_history, belief);
//             }
//         }
//     }

//     void BaseOccupancyState::setProbabilityOverIndividualHistories()
//     {
//         for (number ag_id = 0; ag_id < this->num_agents_; ag_id++)
//         {
//             for (const auto &ihistory : this->getIndividualHistories(ag_id))
//             {
//                 double prob = 0;
//                 for (const auto &pair_history_hidden_state : *this->getPrivateOccupancyState(ag_id, ihistory))
//                 {
//                     prob += pair_history_hidden_state.second;
//                 }
//                 this->probability_ihistories[ag_id][ihistory] = prob;
//             }
//         }
//     }

//     void BaseOccupancyState::setup()
//     {
//         this->setupBeliefsAndHistories();
//         this->setupIndividualHistories();
//     }

//     void BaseOccupancyState::finalize()
//     {
//         this->setup();
//         this->setProbabilityOverJointHistory();
//         this->setProbabilityOverIndividualHistories();
//     }

//     TypeState BaseOccupancyState::getTypeState() const
//     {
//         return TypeState::OCCUPANCY_STATE;
//     }

//     std::shared_ptr<OccupancyState> BaseOccupancyState::getptr()
//     {
//         return std::static_pointer_cast<OccupancyState>(this->toState()->toOccupancyState());
//     }

//     std::string BaseOccupancyState::str() const
//     {
//         std::ostringstream res;
//         res << std::setprecision(config::OCCUPANCY_DECIMAL_PRINT) << std::fixed;

//         res << "<occupancy-state size=\"" << MappedVector<std::shared_ptr<State>>::size() << "\">\n";
//         for (const auto &pair_state_proba : *this)
//         {
//             auto history = std::static_pointer_cast<JointHistoryBeliefPair>(pair_state_proba.first)->first;
//             auto belief = std::static_pointer_cast<JointHistoryBeliefPair>(pair_state_proba.first)->second;

//             res << "\t<probability";
//             res << " history=" << history->short_str() << "";
//             res << " belief=" << belief->str() << ">\n";
//             res << "\t\t\t" << pair_state_proba.second << "\n";
//             res << "\t</probability \n";
//         }
//         res << "</occupancy-state>";
//         return res.str();
//     }

//     // #############################################
//     // ######### ACTION SPACE ######################
//     // #############################################

//     std::shared_ptr<Space> BaseOccupancyState::getActionSpaceAt(number t)
//     {
//         if (this->action_space_map->find(t) != this->action_space_map->end())
//         {
//             return this->action_space_map->at(t);
//         }
//         else
//         {
//             return nullptr;
//         }
//     }

//     void BaseOccupancyState::setActionSpaceAt(number t, std::shared_ptr<Space> action_space)
//     {
//         this->action_space_map->emplace(t, action_space);
//     }

// } // namespace sdm
