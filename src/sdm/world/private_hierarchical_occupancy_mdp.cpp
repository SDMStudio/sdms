#include <sdm/world/private_hierarchical_occupancy_mdp.hpp>

namespace sdm
{

    PrivateHierarchicalOccupancyMDP::PrivateHierarchicalOccupancyMDP()
    {
    }

    PrivateHierarchicalOccupancyMDP::PrivateHierarchicalOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &underlying_dpomdp, number memory, bool compression, bool store_states, bool store_actions, int batch_size)
        : OccupancyMDP(underlying_dpomdp, memory, compression, store_states, store_actions, batch_size)
    {
    }

    std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> PrivateHierarchicalOccupancyMDP::step(std::shared_ptr<Action> action)
    {
        clock_t t_begin = clock(), t_tmp = clock();
        this->current_action_ = this->applyDecisionRule(this->current_state_->toOccupancyState(), this->current_history_->toJointHistory(), action, this->step_);
        OccupancyMDP::TIME_IN_APPLY_DR += ((float)(clock() - t_tmp) / CLOCKS_PER_SEC);

        t_tmp = clock();
        auto [observation, rewards, is_done] = this->getUnderlyingProblem()->step(this->current_action_);
        OccupancyMDP::TIME_IN_UNDER_STEP += ((float)(clock() - t_tmp) / CLOCKS_PER_SEC);

        t_tmp = clock();
        double occupancy_reward = this->getReward(this->current_state_, action, this->step_);
        OccupancyMDP::TIME_IN_GET_REWARD += ((float)(clock() - t_tmp) / CLOCKS_PER_SEC);

        // std::cout << "occupancy_reward " << occupancy_reward << std::endl;
        std::shared_ptr<Observation> observation_n = std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(observation)->at(this->getUnderlyingMDP()->getNumAgents() - 1);

        t_tmp = clock();
        this->current_state_ = this->nextOccupancyState(this->current_state_, action, observation_n, this->step_);
        OccupancyMDP::TIME_IN_NEXT_OSTATE += ((float)(clock() - t_tmp) / CLOCKS_PER_SEC);

        this->current_history_ = this->getNextHistory(observation);
        this->step_++;

        OccupancyMDP::TIME_IN_STEP += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);

        return std::make_tuple(this->current_state_, std::vector<double>{occupancy_reward, rewards[0]}, is_done);
        
    }

    std::shared_ptr<Space> PrivateHierarchicalOccupancyMDP::computeActionSpaceAt(const std::shared_ptr<State> &ostate, number t)
    {
        // std::cout << "computeActionSpaceAt()" << std::endl;
        // Get joint histories that are in the support of the occupancy state.
        std::set<std::shared_ptr<JointHistoryInterface>> joint_histories = ostate->toOccupancyState()->getJointHistories();
        // Vector for individual hierarchical histories of all agents from 1 to N.
        std::vector<std::set<std::shared_ptr<JointHistoryInterface>>> all_individual_hierarchical_histories;
        // For all agents from 1 to N:
        for (int agent = 0; agent < this->getUnderlyingProblem()->getNumAgents(); agent++)
        {
            // std::cout << "agent " << agent << std::endl;
            // Individual hierarchical histories for Agent I.
            std::set<std::shared_ptr<JointHistoryInterface>> individual_hierarchical_histories;
            // For each possible joint history:
            for (std::shared_ptr<JointHistoryInterface> joint_history : joint_histories)
            {
                //
                std::shared_ptr<JointHistoryInterface> individual_hierarchical_history = std::make_shared<JointHistoryTree>();
                // For each agent between agent I and agent N (both included):
                for (int lower_ranked_agent = agent; lower_ranked_agent < this->getUnderlyingProblem()->getNumAgents(); lower_ranked_agent++)
                {
                    // std::cout << "lower_ranked_agent " << lower_ranked_agent << std::endl;
                    std::shared_ptr<HistoryInterface> individual_history = joint_history->getIndividualHistory(lower_ranked_agent);
                    // std::cout << "*individual_history " << *individual_history << std::endl;
                    individual_hierarchical_history->addIndividualHistory(individual_history);
                }
                // std::cout << "*individual_hierarchical_history " << *individual_hierarchical_history << std::endl;
                // std::cout << "individual_hierarchical_history->getNumAgents() " << std::dynamic_pointer_cast<JointHistoryTree>(individual_hierarchical_history)->getNumAgents() << std::endl;
                // this->individual_hierarchical_history_map->emplace(*individual_hierarchical_history, individual_hierarchical_history);
                ostate->toOccupancyState()->pushToIndividualHierarchicalHistoriesOf(t, agent, individual_hierarchical_history);
                individual_hierarchical_histories.emplace(individual_hierarchical_history);
            }
            all_individual_hierarchical_histories.push_back(individual_hierarchical_histories);
        }

        // Vector of individual deterministic decision rules of each agent.
        std::vector<std::shared_ptr<Space>> individual_ddr_spaces;
        // For each agent from 1 to N:
        for (int agent = 0; agent < this->getUnderlyingProblem()->getNumAgents(); agent++)
        {
            // Get individual hierarchical histories of agent I.
            std::set<std::shared_ptr<JointHistoryInterface>> individual_hierarchical_histories = all_individual_hierarchical_histories[agent];
            // Get individual hierarchical history space of agent I.
            std::shared_ptr<Space> individual_hierarchical_history_space = std::make_shared<DiscreteSpace>(sdm::tools::set2vector(individual_hierarchical_histories));
            // Get action space of agent I.
            std::shared_ptr<Space> individual_action_space = std::static_pointer_cast<MultiDiscreteSpace>(this->getUnderlyingProblem()->getActionSpace(t))->get(agent);
            // Get individual ddr of agent I.
            std::shared_ptr<Space> individual_ddr_space = std::make_shared<FunctionSpace<DeterministicDecisionRule>>(individual_hierarchical_history_space, individual_action_space, this->store_actions_);
            // Add it to the corresponding vector.
            individual_ddr_spaces.push_back(individual_ddr_space);
        }

        // Create the function space of joint deterministic decision rules.
        std::shared_ptr<Space> joint_ddr_space = std::make_shared<FunctionSpace<JointDeterministicDecisionRule>>(
            std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{std::make_shared<DiscreteState>(3)}),
            std::make_shared<MultiDiscreteSpace>(individual_ddr_spaces, this->store_actions_),
            this->store_actions_);
        return joint_ddr_space;
    }

    Pair<std::shared_ptr<State>, std::shared_ptr<State>> PrivateHierarchicalOccupancyMDP::computeExactNextState(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation_n, number t)
    {
        auto occupancy_state = ostate->toOccupancyState();
        auto decision_rule = action->toDecisionRule();
        OccupancyMDP::MEAN_SIZE_STATE += occupancy_state->getFullyUncompressedOccupancy()->getStates().size();

        OccupancyMDP::PASSAGE_IN_NEXT_STATE++;
        clock_t t_begin = clock();

        // The new fully uncompressed occupancy state
        std::shared_ptr<OccupancyStateInterface> fully_uncompressed_next_occupancy_state = std::make_shared<OccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());
        // The new one step left occupancy state
        std::shared_ptr<OccupancyStateInterface> one_step_left_compressed_next_occupancy_state = std::make_shared<OccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());

        // For each joint history in the support of the fully uncompressed occupancy state
        for (const auto &joint_history : occupancy_state->getFullyUncompressedOccupancy()->getJointHistories())
        {
            // Apply the joint decision rule at joint_history to get the joint_action
            auto joint_action = this->applyDecisionRule(occupancy_state->toOccupancyState(), joint_history, decision_rule, t);

            // For each accessible belief at joint_history
            auto belief = occupancy_state->getFullyUncompressedOccupancy()->getBeliefAt(joint_history);

            // For each observation in
            for (auto &joint_observation : *this->getUnderlyingMPOMDP()->getObservationSpace(t))
            {
                if (std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(joint_observation)->at(this->getUnderlyingMDP()->getNumAgents() - 1) == observation_n)
                {
                    // Get the next joint history
                    auto next_joint_history = joint_history->expand(joint_observation->toObservation());

                    // Get the next belief
                    auto next_belief = this->getUnderlyingBeliefMDP()->nextBelief(belief, joint_action, joint_observation->toObservation(), t);

                    // Compute the probability of next history, i.e. p(o') = p(o) * p(z | b, a)
                    double next_joint_history_probability = occupancy_state->getFullyUncompressedOccupancy()->getProbability(joint_history) * this->getUnderlyingBeliefMDP()->getObservationProbability(belief, joint_action, next_belief->toBelief(), joint_observation->toObservation(), t);

                    // If the next history probability is not zero
                    if (next_joint_history_probability > 0)
                    {
                        // Build fully uncompressed occupancy state
                        fully_uncompressed_next_occupancy_state->addProbability(next_joint_history->toJointHistory(), next_belief->toBelief(), next_joint_history_probability);

                        // Update the probability of being in this next history (for the one step left uncompressed occupancy state)
                        auto compressed_joint_history = occupancy_state->getCompressedJointHistory(joint_history);
                        auto next_compressed_joint_history = compressed_joint_history->expand(joint_observation->toObservation());
                        one_step_left_compressed_next_occupancy_state->addProbability(next_compressed_joint_history->toJointHistory(), next_belief->toBelief(), next_joint_history_probability);

                        // Update next history labels
                        one_step_left_compressed_next_occupancy_state->updateJointLabels(next_joint_history->toJointHistory()->getIndividualHistories(), next_compressed_joint_history->toJointHistory()->getIndividualHistories());
                    }
                }

            }
        }

        fully_uncompressed_next_occupancy_state->finalize();
        one_step_left_compressed_next_occupancy_state->finalize();

        OccupancyMDP::TIME_IN_NEXT_STATE += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);

        return std::make_pair(fully_uncompressed_next_occupancy_state, one_step_left_compressed_next_occupancy_state);
    }

    Pair<std::shared_ptr<State>, double> PrivateHierarchicalOccupancyMDP::computeNextStateAndProbability(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        //
        std::shared_ptr<State> next_state = this->computeNextState(belief, action, observation, t);
        // Compute the coefficient of normalization (eta)
        double eta = next_state->toBelief()->norm_1();
        next_state->toBelief()->normalizeBelief(eta);
        return {next_state->toBelief(), eta};
    }

    Pair<std::shared_ptr<State>, std::shared_ptr<State>> PrivateHierarchicalOccupancyMDP::computeSampledNextState(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation_n, number t)
    {
        // The new fully uncompressed occupancy state
        std::shared_ptr<OccupancyStateInterface> fully_uncompressed_next_occupancy_state = std::make_shared<OccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());
        // The new one step left occupancy state
        std::shared_ptr<OccupancyStateInterface> one_step_left_compressed_next_occupancy_state = std::make_shared<OccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());

        // Bag for next history (w) and next state (y) and counters for the two.
        std::unordered_map<std::shared_ptr<HistoryInterface>, Pair<double, std::unordered_map<std::shared_ptr<State>, double>>> w_y_bag;

        // This map is for keeping track of the joint history and next observation for each next joint history.
        std::unordered_map<std::shared_ptr<HistoryInterface>, Tuple<std::shared_ptr<HistoryInterface>, std::shared_ptr<BeliefInterface>, std::shared_ptr<Action>, std::shared_ptr<Observation>>> w__o_b_u_z__map;

        //
        std::shared_ptr<State> true_state = this->getUnderlyingProblem()->getInternalState();
        //
        int k = 0;

        while (k < this->batch_size_)
        {
            // Sample joint history and belief.
            auto [joint_history, belief] = ostate->toOccupancyState()->sampleJointHistoryBelief();
            // Sample state.
            auto state = belief->sampleState();
            // Get joint action.
            auto joint_action = this->applyDecisionRule(ostate->toOccupancyState()->toOccupancyState(), joint_history, action, t);
            // Set state.
            this->getUnderlyingProblem()->setInternalState(state);
            // Sample next observation.
            auto [observation, _, __] = this->getUnderlyingProblem()->step(joint_action, false);
            if (std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(observation)->at(this->getUnderlyingMDP()->getNumAgents() - 1) == observation_n)
            {
                // Sample next state.
                auto next_state = this->getUnderlyingProblem()->getInternalState();
                // Sample next joint history.
                auto next_joint_history = joint_history->expand(observation);
                // If next_joint_history is seen for the first time:
                if (w_y_bag.find(next_joint_history) == w_y_bag.end())
                {
                    // Set next_joint_history's counter to 1.
                    w_y_bag[next_joint_history].first = 1;
                    // No need to check if (next_joint_history, next_state) was seen before, the answer is no.
                    // Set (next_joint_history, next_state)'s counter to 1.
                    w_y_bag[next_joint_history].second[next_state] = 1;
                    // These two will be needed later.
                    w__o_b_u_z__map[next_joint_history] = std::make_tuple(joint_history, belief, joint_action, observation);
                }
                // If next_joint_history was seen before:
                else
                {
                    // Incriment next_joint_history's counter.
                    w_y_bag[next_joint_history].first++;
                    // If (next_joint_history, next_state) is seen for the first time:
                    if (w_y_bag[next_joint_history].second.find(next_state) == w_y_bag[next_joint_history].second.end())
                    {
                        // Set (next_joint_history, next_state)'s counter to 1.
                        w_y_bag[next_joint_history].second[next_state] = 1;
                    }
                    // If (next_joint_history, next_state) was seen before:
                    else
                    {
                        // Incriment (next_joint_history, next_state)'s counter.
                        w_y_bag[next_joint_history].second[next_state]++;
                    }
                }
                k++;
            }
        }

        // Iterate through the (next_joint_history, next_state) pairs.
        for (auto const [next_joint_history, next_joint_history_count__y_bag] : w_y_bag)
        {
            auto [next_joint_history_count, y_bag] = next_joint_history_count__y_bag;
            auto [joint_history, belief, joint_action, observation] = w__o_b_u_z__map[next_joint_history];
            std::shared_ptr<BeliefInterface> next_belief;
            auto belief_graph = this->belief_mdp_->getMDPGraph();
            auto successor = belief_graph->getNode(belief)->getSuccessor(std::make_pair(joint_action, observation));
            // If already in the successor list
            if (successor != nullptr)
            {
                // Return the successor node
                next_belief = successor->getData()->toBelief();
            }
            else
            {
                next_belief = std::make_shared<Belief>();

                for (auto const [next_state, next_state_count] : y_bag)
                {
                    next_belief->setProbability(next_state, next_state_count / next_joint_history_count);
                }
            }
            next_belief->finalize();
            belief_graph->addNode(next_belief);
            fully_uncompressed_next_occupancy_state->addProbability(next_joint_history->toJointHistory(), next_belief, next_joint_history_count / this->batch_size_);
            auto joint_history_ = next_joint_history->getPreviousHistory();
            auto observation_ = next_joint_history->getObservation();
            auto compressed_joint_history = ostate->toOccupancyState()->getCompressedJointHistory(joint_history->toJointHistory());
            auto next_compressed_joint_history = compressed_joint_history->expand(observation->toObservation());
            one_step_left_compressed_next_occupancy_state->addProbability(next_compressed_joint_history->toJointHistory(), next_belief->toBelief(), next_joint_history_count / this->batch_size_);
            one_step_left_compressed_next_occupancy_state->updateJointLabels(next_joint_history->toJointHistory()->getIndividualHistories(), next_compressed_joint_history->toJointHistory()->getIndividualHistories());
        }
        //
        this->getUnderlyingProblem()->setInternalState(true_state);

        // Finalize the one step left compressed occupancy state
        one_step_left_compressed_next_occupancy_state->finalize();
        fully_uncompressed_next_occupancy_state->finalize();

        return std::make_pair(fully_uncompressed_next_occupancy_state, one_step_left_compressed_next_occupancy_state);
    }

    std::shared_ptr<Action> PrivateHierarchicalOccupancyMDP::applyDecisionRule(const std::shared_ptr<OccupancyStateInterface> &ostate, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<Action> &decision_rule, number t) const
    {
        // std::cout << std::endl;
        // std::cout << "applyDecisionRule()" << std::endl;
        // Get list of individual history labels
        auto joint_labels = ostate->toOccupancyState()->getJointLabels(joint_history->getIndividualHistories()).toJoint<State>();

        auto joint_hierarchical_labels = this->getJointHierarchicalLabels(joint_labels, ostate);



        // std::cout << "*decision_rule " << std::endl;
        // std::cout << *decision_rule << std::endl;
        // std::cout << "*ostate " << std::endl;
        // std::cout << *ostate << std::endl;
        // std::cout << "*joint_history " << std::endl;
        // std::cout << *joint_history << std::endl;
        // std::cout << "*joint_labels " << std::endl;
        // std::cout << *joint_labels << std::endl;
        // std::cout << "*joint_hierarchical_labels " << std::endl;
        // std::cout << *joint_hierarchical_labels << std::endl;



        // std::cout << std::endl;

        // Get selected joint action
        auto action = std::static_pointer_cast<JointDeterministicDecisionRule>(decision_rule)->act(joint_hierarchical_labels);

        // std::cout << std::endl;

        // std::cout << "*action " << *action << std::endl;

        // std::cout << std::endl;

        // Transform selected joint action into joint action address
        auto joint_action = std::static_pointer_cast<Joint<std::shared_ptr<Action>>>(action);
        // Get the adress of the joint action object from the space of available joint action object.
        auto joint_action_address = std::static_pointer_cast<MultiDiscreteSpace>(this->getUnderlyingProblem()->getActionSpace(t))->getItemAddress(*joint_action->toJoint<Item>());
        return joint_action_address->toAction();
    }


    std::shared_ptr<State> PrivateHierarchicalOccupancyMDP::getJointHierarchicalLabels(const std::shared_ptr<State> &joint_labels, const std::shared_ptr<State> &ostate) const
    {
        // std::cout << "getJointHierarchicalLabels()" << std::endl;
        // std::cout << "*joint_labels" << std::endl;
        // std::cout << *joint_labels << std::endl;

        // This is the reversed version of what we want, that is Joint Hierarchical Labels, that is Hierarchical Labels for each agent.
        // Each Hierarchical Label contains Labels for agents between agent I and agent N.
        std::shared_ptr<Joint<std::shared_ptr<JointHistoryInterface>>> joint_hierarchical_labels_reversed = std::make_shared<Joint<std::shared_ptr<JointHistoryInterface>>>();
        // std::cout << "*joint_hierarchical_labels_reversed " << *joint_hierarchical_labels_reversed << std::endl;
        // This is what we will use to record Labels of each agent, starting with agent N until agent 1. This is why it's reversed.
        std::shared_ptr<JointHistoryInterface> individual_hierarchical_label_reversed = std::make_shared<JointHistoryTree>();
        // std::cout << "*individual_hierarchical_label_reversed " << *individual_hierarchical_label_reversed << std::endl;
        // std::cout << "individual_hierarchical_label_reversed->getNumAgents() " << std::dynamic_pointer_cast<JointHistoryTree>(individual_hierarchical_label_reversed)->getNumAgents() << std::endl;
        // For agent from N-1 till 0 (N till 1):
        for(int agent = this->getUnderlyingMDP()->getNumAgents() - 1; agent >= 0; agent--)
        {
            // std::cout << "agent " << agent << std::endl;
            // Push agent I's Label.
            auto individual_label = std::dynamic_pointer_cast<Joint<std::shared_ptr<State>>>(joint_labels)->get(agent);
            // std::cout << "*individual_label " << *individual_label << std::endl;
            // std::cout << "*individual_label->toHistory() " << *individual_label->toHistory() << std::endl;
            individual_hierarchical_label_reversed->addIndividualHistory(individual_label->toHistory());
            // std::cout << "*individual_hierarchical_label_reversed " << *individual_hierarchical_label_reversed << std::endl;
            // This will be in the correct order, that is Labels for agent I till N.
            std::shared_ptr<JointHistoryInterface> individual_hierarchical_label = std::make_shared<JointHistoryTree>();
            // std::cout << "*individual_hierarchical_label " << *individual_hierarchical_label << std::endl;
            //
            for(int i = std::dynamic_pointer_cast<JointHistoryTree>(individual_hierarchical_label_reversed)->getNumAgents() - 1; i >= 0; i--)
            {
                // std::cout << "i " << i << std::endl;
                // std::cout << "*individual_hierarchical_label_reversed->getIndividualHistory(i) " << *individual_hierarchical_label_reversed->getIndividualHistory(i) << std::endl;
                // 
                individual_hierarchical_label->addIndividualHistory(individual_hierarchical_label_reversed->getIndividualHistory(i));
            }
            //
            for (const std::shared_ptr<JointHistoryInterface>& individual_hierarchical_history: ostate->toOccupancyState()->getIndividualHierarchicalHistoriesOf(0, agent))
            {
                if (*std::dynamic_pointer_cast<JointHistoryTree>(individual_hierarchical_history) == *std::dynamic_pointer_cast<JointHistoryTree>(individual_hierarchical_label))
                {
                    individual_hierarchical_label = individual_hierarchical_history;
                    break;
                }
            }
            // individual_hierarchical_label = this->individual_hierarchical_history_map->at(*std::dynamic_pointer_cast<JointHistoryTree>(individual_hierarchical_label));
            // Push Hierarchical Label for agent I.
            joint_hierarchical_labels_reversed->push_back(individual_hierarchical_label);
        }
        // This will be in the correct order, that is Hierarchical Labels from agent 1 to N.
        std::shared_ptr<Joint<std::shared_ptr<JointHistoryInterface>>> joint_hierarchical_labels = std::make_shared<Joint<std::shared_ptr<JointHistoryInterface>>>();
        // 
        for (int i = joint_hierarchical_labels_reversed->getNumAgents() - 1; i >= 0; i--)
        {
            //
            joint_hierarchical_labels->push_back(joint_hierarchical_labels_reversed->at(i));
        }
        return joint_hierarchical_labels->toJoint<State>();
    }

} // namespace sdm
