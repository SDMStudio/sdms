#include <sdm/world/private_hierarchical_occupancy_mdp_with_observation.hpp>
#include <sdm/core/observation/default_observation.hpp>


namespace sdm
{

    PrivateHierarchicalOccupancyMDPWithObservation::PrivateHierarchicalOccupancyMDPWithObservation()
    {
    }

    PrivateHierarchicalOccupancyMDPWithObservation::PrivateHierarchicalOccupancyMDPWithObservation(const std::shared_ptr<MPOMDPInterface> &underlying_dpomdp, number memory, bool compression, bool store_states, bool store_actions, bool generate_action_spaces, int batch_size)
        : PrivateHierarchicalOccupancyMDPWithHistory(underlying_dpomdp, memory, compression, store_states, store_actions, generate_action_spaces, batch_size)
    {
        
    }

    std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> PrivateHierarchicalOccupancyMDPWithObservation::step(std::shared_ptr<Action> action)
    {
        // std::cout << "PrivateHierarchicalOccupancyMDPWithObservation::step()" << std::endl;

        this->current_action_ = this->applyDecisionRule(this->current_observation_, action, this->step_);

        // std::cout << "*this->current_action_ " << *this->current_action_ << std::endl;

        auto [next_observation, rewards, __] = this->getUnderlyingProblem()->step(this->current_action_);

        // double occupancy_reward = this->getReward(this->current_state_, action, this->step_);

        // std::cout << "*next_observation " << *next_observation << std::endl;

        std::shared_ptr<Observation> next_observation_n = std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(next_observation)->at(this->getUnderlyingMDP()->getNumAgents() - 1);

        // std::cout << "*next_observation_n " << *next_observation_n << std::endl;

        this->current_state_ = this->nextOccupancyState(this->current_state_, action, next_observation_n, this->step_);

        // std::cout << "*this->current_state_ " << std::endl;
        // std::cout << *this->current_state_  << std::endl;

        this->current_history_ = this->getNextHistory(next_observation);
        this->current_observation_ = this->current_history_->getObservation();
        this->step_++;

        std::shared_ptr<PrivateHierarchicalOccupancyStateObservationPair> s_z = std::make_shared<PrivateHierarchicalOccupancyStateObservationPair>(std::make_pair(this->current_state_->toOccupancyState(), this->current_observation_));

        return std::make_tuple(s_z, std::vector<double>{21211, rewards[0]}, (this->step_ > this->getUnderlyingMPOMDP()->getHorizon()));
    }

    std::shared_ptr<Observation> PrivateHierarchicalOccupancyMDPWithObservation::reset()
    {
        // std::cout << "PrivateHierarchicalOccupancyMDPWithObservation::reset()" << std::endl;

        OccupancyMDP::reset();

        this->current_observation_ = nullptr;

        std::shared_ptr<PrivateHierarchicalOccupancyStateObservationPair> s_z = std::make_shared<PrivateHierarchicalOccupancyStateObservationPair>(std::make_pair(this->current_state_->toState()->toOccupancyState(), this->current_observation_));

        return s_z;
    }

    double PrivateHierarchicalOccupancyMDPWithObservation::getReward(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t)
    {
        // std::cout << "PrivateHierarchicalOccupancyMDPWithObservation::getReward()" << std::endl;

        auto joint_label = this->getJointLabel(this->current_history_, occupancy_state);
        auto belief = occupancy_state->toOccupancyState()->getBeliefAt(joint_label->toJointHistory());
        auto joint_action = this->applyDecisionRule(this->current_observation_, decision_rule, t);
        return this->getUnderlyingBeliefMDP()->getReward(belief, joint_action, t);
    }

    std::shared_ptr<Action> PrivateHierarchicalOccupancyMDPWithObservation::getRandomAction(const std::shared_ptr<Observation> &ostate, number t)
    {
        // std::cout << "PrivateHierarchicalOccupancyMDPWithObservation::getRandomAction()" << std::endl;

        auto s = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateObservationPair>(ostate)->first;

        return PrivateHierarchicalOccupancyMDP::getRandomAction(s->toState()->toObservation(), t);
    }

    std::shared_ptr<Action> PrivateHierarchicalOccupancyMDPWithObservation::computeRandomAction(const std::shared_ptr<OccupancyStateInterface> &ostate, number t)
    {
        // std::cout << "PrivateHierarchicalOccupancyMDPWithObservation::computeRandomAction()" << std::endl;

        std::vector<std::shared_ptr<DeterministicDecisionRule>> a;

        // agent 1
        {
            // Input states for the a of agent.
            std::vector<std::shared_ptr<Item>> inputs;
            // Outputed actions for each of these.
            std::vector<std::shared_ptr<Item>> outputs;

            if (t == 0)
            {
                inputs.push_back(NO_OBSERVATION);
                outputs.push_back(std::static_pointer_cast<MultiDiscreteSpace>(this->getUnderlyingProblem()->getActionSpace(t))->get(0)->sample());
            }
            else
            {
                for(auto &z : *this->getUnderlyingMPOMDP()->getObservationSpace(t - 1))
                {
                    inputs.push_back(z);
                    outputs.push_back(std::static_pointer_cast<MultiDiscreteSpace>(this->getUnderlyingProblem()->getActionSpace(t - 1))->get(0)->sample());
                }
            }
            

            a.push_back(std::make_shared<DeterministicDecisionRule>(inputs, outputs));
            // std::cout << *std::make_shared<DeterministicDecisionRule>(inputs, outputs) << std::endl;
        }


        // agent 2
        {
            // Input states for the a of agent.
            std::vector<std::shared_ptr<Item>> inputs;
            // Outputed actions for each of these.
            std::vector<std::shared_ptr<Item>> outputs;
            if (t == 0)
            {
                inputs.push_back(NO_OBSERVATION);
                outputs.push_back(std::static_pointer_cast<MultiDiscreteSpace>(this->getUnderlyingProblem()->getActionSpace(t))->get(1)->sample());
            }
            else
            {
                for(auto &z : *this->getUnderlyingMPOMDP()->getObservationSpace(t - 1))
                {
                    auto z_2 = std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(z)->at(this->getUnderlyingMDP()->getNumAgents() - 1);
                    inputs.push_back(z_2->toItem());
                    outputs.push_back(std::static_pointer_cast<MultiDiscreteSpace>(this->getUnderlyingProblem()->getActionSpace(t))->get(1)->sample());
                }
            }

            

            a.push_back(std::make_shared<DeterministicDecisionRule>(inputs, outputs));
            // std::cout << *std::make_shared<DeterministicDecisionRule>(inputs, outputs) << std::endl;
        }

        return std::make_shared<JointDeterministicDecisionRule>(a);

    }

    std::shared_ptr<Action> PrivateHierarchicalOccupancyMDPWithObservation::applyDecisionRule(const std::shared_ptr<Observation> &observation, const std::shared_ptr<Action> &decision_rule, number t) const
    {
        // std::cout << "PrivateHierarchicalOccupancyMDPWithObservation::applyDecisionRule()" << std::endl;

        // std::cout << "observation" << std::endl;
        // std::cout << observation << std::endl;
        

        

        std::shared_ptr<Joint<std::shared_ptr<Observation>>> joint_hierarchical_observations = std::make_shared<Joint<std::shared_ptr<Observation>>>();
        if (observation == nullptr)
        {
            joint_hierarchical_observations->push_back(NO_OBSERVATION);
            joint_hierarchical_observations->push_back(NO_OBSERVATION);
        }
        else
        {
            // std::cout << "*observation" << std::endl;
            // std::cout << *observation << std::endl;
            std::shared_ptr<Observation> observation_1 = observation;
            std::shared_ptr<Observation> observation_2 = std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(observation)->at(this->getUnderlyingMDP()->getNumAgents() - 1);
            joint_hierarchical_observations->push_back(observation_1);
            joint_hierarchical_observations->push_back(observation_2);
        }
        
        // std::cout << "decision_rule" << std::endl;
        // std::cout << decision_rule << std::endl;
        // std::cout << "*decision_rule" << std::endl;
        // std::cout << *decision_rule << std::endl;

       // Get the selected action
        auto action = std::static_pointer_cast<Joint<std::shared_ptr<Action>>>(std::static_pointer_cast<JointDeterministicDecisionRule>(decision_rule)->act(joint_hierarchical_observations));

        // std::cout << "action" << std::endl;
        // std::cout << action << std::endl;
        // std::cout << "*action" << std::endl;
        // std::cout << *action << std::endl;


        // Get the adress of the action object from the space of available action object.
        return std::static_pointer_cast<MultiDiscreteSpace>(this->getUnderlyingProblem()->getActionSpace(t))->getItemAddress(*action->toJoint<Item>())->toAction();
    }

    Pair<std::shared_ptr<State>, std::shared_ptr<State>> PrivateHierarchicalOccupancyMDPWithObservation::computeExactNextState(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation_n, number t)
    {
        std::cout << "PrivateHierarchicalOccupancyMDPWithObservation::computeExactNextState()" << std::endl;

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
            auto observation = joint_history->getObservation();

            // Apply the joint decision rule at joint_history to get the joint_action
            auto joint_action = this->applyDecisionRule(observation, action, t);

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

    Pair<std::shared_ptr<State>, std::shared_ptr<State>> PrivateHierarchicalOccupancyMDPWithObservation::computeSampledNextState(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation_n, number t)
    {
        // std::cout << "PrivateHierarchicalOccupancyMDPWithObservation::computeSampledNextState() " << std::endl;
        // std::cout << "PrivateHierarchicalOccupancyMDPWithObservation::computeSampledNextState() 0" << std::endl;

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
            // std::cout << "k " << k << std::endl;
            // Sample joint history and belief.
            auto [joint_history, belief] = ostate->toOccupancyState()->sampleJointHistoryBelief();
            // Sample state.
            auto state = belief->sampleState();

            auto observation = joint_history->getObservation();

            // Apply the joint decision rule at joint_history to get the joint_action
            auto joint_action = this->applyDecisionRule(observation, action, t);

            // Set state.
            this->getUnderlyingProblem()->setInternalState(state);
            // Sample next observation.
            auto [next_observation, _, __] = this->getUnderlyingProblem()->step(joint_action, false);
            if (std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(next_observation)->at(this->getUnderlyingMDP()->getNumAgents() - 1) == observation_n)
            {
                // Sample next state.
                auto next_state = this->getUnderlyingProblem()->getInternalState();
                // Sample next joint history.
                auto next_joint_history = joint_history->expand(next_observation);
                // If next_joint_history is seen for the first time:
                if (w_y_bag.find(next_joint_history) == w_y_bag.end())
                {
                    // Set next_joint_history's counter to 1.
                    w_y_bag[next_joint_history].first = 1;
                    // No need to check if (next_joint_history, next_state) was seen before, the answer is no.
                    // Set (next_joint_history, next_state)'s counter to 1.
                    w_y_bag[next_joint_history].second[next_state] = 1;
                    // These two will be needed later.
                    w__o_b_u_z__map[next_joint_history] = std::make_tuple(joint_history, belief, joint_action, next_observation);
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
        // std::cout << "PrivateHierarchicalOccupancyMDPWithObservation::computeSampledNextState() 1" << std::endl;

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
        // std::cout << "PrivateHierarchicalOccupancyMDPWithObservation::computeSampledNextState() 2" << std::endl;

        // Finalize the one step left compressed occupancy state
        one_step_left_compressed_next_occupancy_state->finalize();
        fully_uncompressed_next_occupancy_state->finalize();
        // std::cout << "PrivateHierarchicalOccupancyMDPWithObservation::computeSampledNextState() 3" << std::endl;

        return std::make_pair(fully_uncompressed_next_occupancy_state, one_step_left_compressed_next_occupancy_state);
    }

} // namespace sdm
