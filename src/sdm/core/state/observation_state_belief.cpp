// #include <iomanip>
// #include <sdm/config.hpp>
// #include <sdm/exception.hpp>
// #include <sdm/core/state/observation_state_belief.hpp>

// #include <algorithm>

// namespace sdm
// {

//     ObservationStateBelief::ObservationStateBelief() : ObservationStateBelief(1)
//     {
//     }

//     ObservationStateBelief::ObservationStateBelief(number num_agents) : Belief(num_agents), num_agents_(num_agents)
//     {

//     }

//     ObservationStateBelief::ObservationStateBelief(const ObservationStateBelief &occupancy_state)
//         : Belief(occupancy_state),
//           num_agents_(occupancy_state.num_agents_),
//           list_beliefs_(occupancy_state.list_beliefs_)
//     {
//     }

//     double ObservationStateBelief::getProbability(const std::shared_ptr<State> &observation) const
//     {
//         return Belief::getProbability(observation);
//     }

//     double ObservationStateBelief::getProbability(const std::shared_ptr<Observation> &observation) const
//     {
//         return Belief::getProbability(observation->toState());
//     }

//     double ObservationStateBelief::getProbability(const std::shared_ptr<Observation> &observation, const std::shared_ptr<State> &state) const
//     {
//         // Get the probability p(x,o) = p(o) * b(x | o)
//         clock_t t_begin = clock();
//         auto belief = this->getBeliefAt(observation);
//         auto output = (belief == nullptr) ? this->getDefault() : this->getProbability(observation) * belief->getProbability(state);
//         ObservationStateBelief::TIME_IN_GET_PROBA += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);
//         return output;
//         // return this->getProbability(observation) * this->getBeliefAt(observation)->getProbability(state);
//     }

//     void ObservationStateBelief::setProbability(const std::shared_ptr<State> &observation, double proba)
//     {
//         Belief::setProbability(observation, proba);
//     }

//     void ObservationStateBelief::setProbability(const std::shared_ptr<Observation> &observation, const std::shared_ptr<BeliefInterface> &belief, double proba)
//     {
//         clock_t t_begin = clock();
//         // Set the belief corresponding to a specific joint history
//         this->setBeliefAt(observation, belief);
//         // Set the probability of the joint history
//         Belief::setProbability(observation, proba);
//         // this->setProbability(observation, proba);
//         ObservationStateBelief::TIME_IN_SET_PROBA += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);
//     }

//     void ObservationStateBelief::addProbability(const std::shared_ptr<State> &observation, double proba)
//     {
//         // Add the probability of being in a joint history
//         this->setProbability(observation, this->getProbability(observation) + proba);

//     }

//     void ObservationStateBelief::addProbability(const std::shared_ptr<Observation> &observation, const std::shared_ptr<BeliefInterface> &belief, double proba)
//     {
//         clock_t t_begin = clock();
//         // Get the corresponding belief of an history. This will return nullptr if no such history exists
//         auto corresponding_belief = this->getBeliefAt(observation);

//         // Get the belief label (corresponding belief or inputed belief)
//         auto belief_label = (corresponding_belief != nullptr) ? corresponding_belief : belief;

//         // Add input probability to the current probability
//         this->setProbability(observation, belief_label, this->getProbability(observation) + proba);
//         ObservationStateBelief::TIME_IN_ADD_PROBA += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);
//     }

//     Pair<std::shared_ptr<Observation>, std::shared_ptr<BeliefInterface>> ObservationStateBelief::sampleJointHistoryBelief()
//     {
//         auto sampled_joint_history = this->distribution_->sample()->toHistory()->toJointHistory();
//         return std::make_pair(sampled_joint_history, this->getBeliefAt(sampled_joint_history));
//     }

//     size_t ObservationStateBelief::hash() const
//     {
//         return std::hash<ObservationStateBelief>()(*this);
//     }

//     bool ObservationStateBelief::operator==(const ObservationStateBelief &other) const
//     {
//         clock_t t_begin = clock();
//         if (this->size() != other.size())
//         {
//             ObservationStateBelief::TIME_IN_EQUAL_OPERATOR += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);
//             return false;
//         }

//         if (std::abs(this->getDefault() - other.getDefault()) > PRECISION)
//         {
//             ObservationStateBelief::TIME_IN_EQUAL_OPERATOR += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);
//             return false;
//         }

//         // For all points in the support
//         for (const auto &jhistory : this->getJointHistories())
//         {
//             // if (this->getBeliefAt(jhistory)->size() != other.getBeliefAt(jhistory)->size())
//             // {
//             //     return false;
//             // }

//             // For all states in the corresponding belief
//             for (const auto &state : this->getBeliefAt(jhistory)->getStates())
//             {
//                 // Does the corresponding probabilities are equals ?
//                 if (std::abs(this->getProbability(jhistory, state) - other.getProbability(jhistory, state)) > ObservationStateBelief::PRECISION)
//                 {
//                     ObservationStateBelief::TIME_IN_EQUAL_OPERATOR += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);
//                     return false;
//                 }
//             }
//         }
//         ObservationStateBelief::TIME_IN_EQUAL_OPERATOR += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);
//         return true;
//     }

//     bool ObservationStateBelief::operator==(const std::shared_ptr<State> &other) const
//     {
//         return this->operator==(*std::dynamic_pointer_cast<ObservationStateBelief>(other));
//     }

//     bool ObservationStateBelief::operator==(const std::shared_ptr<BeliefInterface> &other) const
//     {
//         return ObservationStateBelief::operator==(*std::dynamic_pointer_cast<ObservationStateBelief>(other));
//     }

//     double ObservationStateBelief::operator<(const ObservationStateBelief &other) const
//     {
//         // std::cout<<this->str()<<std::endl;
//         for (const auto &jhistory : this->getJointHistories())
//         {
//             // For all states in the corresponding belief
//             for (const auto &state : this->getBeliefAt(jhistory)->getStates())
//             {
//                 // std::cout<<"jhistory Mine "<<jhistory->str()<<std::endl;
//                 // std::cout<<"state Mine "<<state->str()<<std::endl;

//                 // std::cout<<"Value Mine "<<this->getProbability(jhistory,state)<<std::endl;
//                 // std::cout<<"Value Other "<< other.getProbability(jhistory,state)<<std::endl;
//                 if (this->getProbability(jhistory, state) > other.getProbability(jhistory, state))
//                 {
//                     return false;
//                 }
//             }
//         }

//         for (const auto &jhistory : other.getJointHistories())
//         {
//             // For all states in the corresponding belief
//             for (const auto &state : other.getBeliefAt(jhistory)->getStates())
//             {
//                 // std::cout<<"jhistory Other "<<jhistory->str()<<std::endl;
//                 // std::cout<<"state Other "<<state->str()<<std::endl;

//                 // std::cout<<"Value Mine "<<this->getProbability(jhistory,state)<<std::endl;
//                 // std::cout<<"Value Other "<< other.getProbability(jhistory,state)<<std::endl;
//                 if (other.getProbability(jhistory, state) < this->getProbability(jhistory, state))
//                 {
//                     return false;
//                 }
//             }
//         }
//         return true;
//     }

//     double ObservationStateBelief::operator<(const std::shared_ptr<BeliefInterface> &other) const
//     {
//         return this->operator<(*std::dynamic_pointer_cast<ObservationStateBelief>(other->toOccupancyState()));
//     }

//     double ObservationStateBelief::operator-(const std::shared_ptr<BeliefInterface> &other) const
//     {
//         clock_t t_begin = clock();
//         double distance = 0;
//         std::set<std::shared_ptr<Observation>> this_jhistories = this->getJointHistories();
//         std::set<std::shared_ptr<Observation>> other_jhistories = other->toOccupancyState()->getJointHistories();
//         std::set<std::shared_ptr<Observation>> all_jhistories;
//         std::set_union(std::begin(this_jhistories), std::end(this_jhistories), std::begin(other_jhistories), std::end(other_jhistories), std::inserter(all_jhistories, std::begin(all_jhistories)));
//         // For all joint histories
//         for (const auto &jhistory : all_jhistories)
//         {   
//             // For all states in the corresponding belief
//             for (const auto &state : this->getBeliefAt(jhistory)->getStates())
//             {
//                 // Add the distance
//                 distance += std::abs(this->getProbability(jhistory, state) - other->toOccupancyState()->getProbability(jhistory, state));
//             }
//         }
//         ObservationStateBelief::TIME_IN_MINUS_OPERATOR += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);
//         return distance;
//     }

//     double ObservationStateBelief::minus(const std::shared_ptr<BeliefInterface> &other) const
//     {
//         // std::cout << "ObservationStateBelief::minus()" << std::endl;
//         clock_t t_begin = clock();
//         double distance = 0;
//         // For all joint histories in this
//         for (const auto &jhistory : this->getJointHistories())
//         {   
//             // For all states in the corresponding belief
//             for (const auto &state : this->getBeliefAt(jhistory)->getStates())
//             {
//                 // Add the distance
//                 distance += std::abs(this->getProbability(jhistory, state) - other->toOccupancyState()->getProbability(jhistory, state));
//             }
//         }
//         // For all joint histories in other
//         for (const auto &jhistory : other->toOccupancyState()->getJointHistories())
//         {   
//             // For all states in the corresponding belief
//             for (const auto &state : other->toOccupancyState()->getBeliefAt(jhistory)->getStates())
//             {
//                 // Add the distance
//                 distance += std::abs(this->getProbability(jhistory, state) - other->toOccupancyState()->getProbability(jhistory, state));
//             }
//         }
//         ObservationStateBelief::TIME_IN_MINUS_OPERATOR += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);
//         return distance;
//     }

//     double ObservationStateBelief::operator^(const std::shared_ptr<BeliefInterface> &other) const
//     {
//         double product = 0;

//         for (const auto &jhistory : this->getJointHistories())
//         {
//             for (const auto &state : this->getBeliefAt(jhistory)->getStates())
//             {
//                 product += this->getProbability(jhistory, state) * other->toOccupancyState()->getProbability(jhistory, state);
//             }
//         }
//         return product;
//     }

//     // ###################################
//     // ###### MANIPULATE DATA ############
//     // ###################################

//     const std::set<std::shared_ptr<Observation>> &ObservationStateBelief::getJointHistories() const
//     {
//         return this->list_joint_histories_;
//     }

//     const std::set<std::shared_ptr<BeliefInterface>> &ObservationStateBelief::getBeliefs() const
//     {
//         return this->list_beliefs_;
//     }

//     std::shared_ptr<BeliefInterface> ObservationStateBelief::getBeliefAt(const std::shared_ptr<Observation> &jhistory) const
//     {
//         auto iterator_on_belief = this->map_joint_history_to_belief_.find(jhistory);
//         return (iterator_on_belief == this->map_joint_history_to_belief_.end()) ? nullptr : iterator_on_belief->second;
//     }

//     void ObservationStateBelief::setBeliefAt(const std::shared_ptr<Observation> &jhistory, const std::shared_ptr<BeliefInterface> &belief)
//     {
//         this->map_joint_history_to_belief_[jhistory] = belief;
//     }

//     const std::set<std::shared_ptr<HistoryInterface>> &ObservationStateBelief::getIndividualHistories(number agent_id) const
//     {
//         return this->all_list_ihistories_[agent_id];
//     }

//     const std::vector<std::set<std::shared_ptr<HistoryInterface>>> &ObservationStateBelief::getAllIndividualHistories() const
//     {
//         return this->all_list_ihistories_;
//     }

//     TypeState ObservationStateBelief::getTypeState() const
//     {
//         return TypeState::OCCUPANCY_STATE;
//     }

//     void ObservationStateBelief::setupIndividualHistories()
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

//     void ObservationStateBelief::setupBeliefsAndHistories()
//     {
//         // Get the set of joint histories that are in the support of the ObservationStateBelief
//         this->list_joint_histories_.clear();
//         this->list_beliefs_.clear();
//         for (const auto &joint_history_tmp : this->getStates())
//         {
//             auto observation = joint_history_tmp->toHistory()->toJointHistory();
//             this->list_joint_histories_.insert(observation);
//             this->list_beliefs_.insert(this->getBeliefAt(observation));
//         }
//     }

//     void ObservationStateBelief::setup()
//     {
//         this->setupBeliefsAndHistories();
//         this->setupIndividualHistories();
//     }

//     void ObservationStateBelief::normalize()
//     {
//         double sum = this->norm_1();
//         for (const auto &observation : this->getObservations())
//         {
//             this->setProbability(observation, this->getBeliefAt(observation), this->getProbability(observation) / sum);
//         }
//     }

//     std::shared_ptr<ObservationStateBelief> ObservationStateBelief::getptr()
//     {
//         return std::dynamic_pointer_cast<ObservationStateBelief>(this->toState()->toOccupancyState());
//     }

//     // std::string ObservationStateBelief::str() const
//     // {
//     //     std::ostringstream res;
//     //     res << std::setprecision(config::OCCUPANCY_DECIMAL_PRINT) << std::fixed;

//     //     res << "<occupancy-state defaut=\"" << this->getDefault() << "\" \t size=\"" << MappedVector<std::shared_ptr<State>>::size() << "\">\n";
//     //     for (const auto &history_as_state : this->getIndexes())
//     //     {
//     //         auto observation = history_as_state->toHistory()->toJointHistory();
//     //         res << "\t<probability";
//     //         res << " observation=" << observation->short_str() << "";
//     //         res << " belief=" << this->getBeliefAt(observation)->str() << ">\n";
//     //         res << "\t\t\t" << this->getProbability(observation) << "\n";
//     //         res << "\t</probability \n";
//     //     }
//     //     res << "</occupancy-state>";
//     //     return res.str();
//     // }

// } // namespace sdm
