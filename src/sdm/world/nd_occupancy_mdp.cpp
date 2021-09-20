
// #include <sdm/world/nd_occupancy_mdp.hpp>

// namespace sdm
// {

//     NDOccupancyMDP()
//     {
//     }

//     NDOccupancyMDP(const std::shared_ptr<NDPOMDPInterface> &ndpomdp, number memory, bool compression, bool store_states, bool store_actions, int batch_size)
//     {
//         this->store_states_ = store_states;
//         this->store_actions_ = store_actions;
//         this->batch_size_ = batch_size;
//         this->underlying_problem_ = underlying_dpomdp;

//         // Initialize underlying belief mdp
//         this->belief_mdp_ = std::make_shared<BeliefMDP>(underlying_dpomdp, batch_size);

//         // Initialize initial history
//         // this->initial_history_ = std::make_shared<JointHistoryTree>(this->getUnderlyingMPOMDP()->getNumAgents(), (memory > 0) ? memory : -1);

//         // Initialize initial occupancy state
//         this->initial_state_ = std::make_shared<JointBelief>(this->getUnderlyingMPOMDP()->getNumAgents());
//         for (number agent_id = 0; agent_id < this->getUnderlyingMPOMDP()->getNumAgents(); agent_id++)
//         {
//             this->initial_state_->get(agent_id)->setProbability(this->belief_mdp_->getInitialState()->toBelief()->toJointBelief()->get(agent_id), 1.);
//         }
//         this->initial_state_->toBelief()->finalize();

//         // Set current occupancy state equal to initial one
//         this->current_state_ = this->initial_state_;

//         // Initialize Transition Graph
//         this->mdp_graph_ = std::make_shared<Graph<std::shared_ptr<State>, Pair<std::shared_ptr<Action>, std::shared_ptr<Observation>>>>();
//         this->mdp_graph_->addNode(this->initial_state_);

//         this->reward_graph_ = std::make_shared<Graph<double, Pair<std::shared_ptr<State>, std::shared_ptr<Action>>>>();
//         this->reward_graph_->addNode(0.0);
//     }

//     Pair<std::shared_ptr<State>, std::shared_ptr<State>> computeExactNextState(const std::shared_ptr<State> &occupancy_state,
//                                                                                const std::shared_ptr<Action> &decision_rule,
//                                                                                const std::shared_ptr<Observation> &observation,
//                                                                                number t)
//     {
//         // Get number of agents
//         number num_agents = this->getUnderlyingProblem()->getNumAgents();

//         // Cast into joint belief
//         auto joint_belief = occupancy_state->toBeliefState()->toJointBelief();

//         // Cast into joint decision rule
//         auto joint_decision_rule = decision_rule->toJointDetDecisionRule();

//         // Instanciate next joint belief
//         auto next_joint_belief = std::make_shared<JointBelief>(num_agents);

//         // For each agent
//         for (number agent_id = 0; agent_id < num_agents; agent_id++)
//         {
//             // Compute next occupancy state
//             auto next_belief_agent_i = this->getSubOccupancyMDP(agent_id)->computeExactNextState(joint_belief->get(agent_id),
//                                                                                                  joint_decision_rule->get(agent_id),
//                                                                                                  observation, t);
//             // Add next belief of agent i in the tuple of next beliefs
//             next_joint_belief->push(next_belief_agent_i);
//         }

//         return {next_joint_belief, nullptr};
//     }

//     double getReward(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t)
//     {
//         // return {next_joint_belief, nullptr};
//     }
// }