#include <sdm/world/occupancy_mdp.hpp>

namespace sdm
{
    double OccupancyMDP::TIME_IN_NEXT_STATE = 0,
           OccupancyMDP::TIME_IN_COMPRESS = 0,
           OccupancyMDP::TIME_IN_GET_ACTION = 0,
           OccupancyMDP::TIME_IN_STEP = 0,
           OccupancyMDP::TIME_IN_UNDER_STEP = 0,
           OccupancyMDP::TIME_IN_GET_REWARD = 0,
           OccupancyMDP::TIME_IN_APPLY_DR = 0,
           OccupancyMDP::TIME_IN_NEXT_OSTATE = 0;

    number OccupancyMDP::PASSAGE_IN_NEXT_STATE = 0;
    unsigned long OccupancyMDP::MEAN_SIZE_STATE = 0;

    OccupancyMDP::OccupancyMDP()
    {
    }

    OccupancyMDP::OccupancyMDP(const std::shared_ptr<MPOMDPInterface> &underlying_dpomdp, number memory, bool compression, bool store_states, bool store_action_spaces, bool store_actions, int batch_size)
        : compression_(compression)
    {
        this->store_states_ = store_states;
        this->store_action_spaces_ = store_action_spaces;
        this->store_actions_ = store_actions;
        this->batch_size_ = batch_size;
        this->underlying_problem_ = underlying_dpomdp;

        // Initialize underlying belief mdp
        this->belief_mdp_ = std::make_shared<BeliefMDP>(underlying_dpomdp, batch_size);

        // Initialize initial history
        this->initial_history_ = std::make_shared<JointHistoryTree>(this->getUnderlyingMPOMDP()->getNumAgents(), (memory > 0) ? memory : -1);

        // Initialize initial occupancy state
        this->initial_state_ = std::make_shared<OccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());

        this->initial_state_->toOccupancyState()->setProbability(this->initial_history_->toJointHistory(), this->belief_mdp_->getInitialState()->toBelief(), 1);

        this->initial_state_->toOccupancyState()->finalize();
        this->initial_state_->toOccupancyState()->setFullyUncompressedOccupancy(this->initial_state_->toOccupancyState());
        this->initial_state_->toOccupancyState()->setOneStepUncompressedOccupancy(this->initial_state_->toOccupancyState());

        // Set current occupancy state equal to initial one
        this->current_state_ = this->initial_state_;

        // Initialize Transition Graph
        this->mdp_graph_ = std::make_shared<Graph<std::shared_ptr<State>, Pair<std::shared_ptr<Action>, std::shared_ptr<Observation>>>>();
        this->mdp_graph_->addNode(this->initial_state_);

        this->reward_graph_ = std::make_shared<Graph<double, Pair<std::shared_ptr<State>, std::shared_ptr<Action>>>>();
        this->reward_graph_->addNode(0.0);
    }

    std::shared_ptr<Observation> OccupancyMDP::reset()
    {
        this->current_history_ = this->initial_history_;
        return BaseBeliefMDP<OccupancyState>::reset();
    }

    std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> OccupancyMDP::step(std::shared_ptr<Action> action)
    {
        clock_t t_begin = clock(), t_tmp = clock();
        auto joint_action = this->applyDecisionRule(this->current_state_->toOccupancyState(), this->current_history_->toJointHistory(), action, this->step_);
        OccupancyMDP::TIME_IN_APPLY_DR += ((float)(clock() - t_tmp) / CLOCKS_PER_SEC);

        t_tmp = clock();
        auto [observation, _, __] = this->getUnderlyingProblem()->step(joint_action);
        OccupancyMDP::TIME_IN_UNDER_STEP += ((float)(clock() - t_tmp) / CLOCKS_PER_SEC);

        double occupancy_reward = this->getReward(this->current_state_, action, this->step_);

        t_tmp = clock();
        this->current_state_ = this->nextOccupancyState(this->current_state_, action, nullptr, this->step_);
        OccupancyMDP::TIME_IN_NEXT_OSTATE += ((float)(clock() - t_tmp) / CLOCKS_PER_SEC);

        this->current_history_ = this->getNextHistory(observation);
        this->step_++;

        OccupancyMDP::TIME_IN_STEP += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);
        return std::make_tuple(this->current_state_, std::vector<double>{occupancy_reward}, (this->step_ > this->getUnderlyingMPOMDP()->getHorizon()));
    }

    std::shared_ptr<Space> OccupancyMDP::getActionSpaceAt(const std::shared_ptr<State> &ostate, number t)
    {
        // std::cout << "OccupancyMDP::getActionSpaceAt() " << std::endl;

        clock_t t_begin = clock();
        // If the action space corresponding to this ostate and t does not exist:
        if (ostate->toOccupancyState()->getActionSpaceAt(t) == nullptr)
        {
            // Compute the action space at this occupancy state and timestep
            std::shared_ptr<Space> joint_ddr_space = this->computeActionSpaceAt(ostate, t);
            // Store the action space for state o
            ostate->toOccupancyState()->setActionSpaceAt(t, joint_ddr_space);
        }
        // Return the action space corresponding to this ostate and t.
        OccupancyMDP::TIME_IN_GET_ACTION += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);
        return ostate->toOccupancyState()->getActionSpaceAt(t);
    }

    // Useless
    std::shared_ptr<Space> OccupancyMDP::getActionSpaceAt(const std::shared_ptr<Observation> &ostate, number t)
    {
        // std::cout << "This is a useless function, so this will never be seen by anyone." << std::endl;
        return this->getActionSpaceAt(ostate->toState(), t);
    }

    std::shared_ptr<Action> OccupancyMDP::getRandomAction(const std::shared_ptr<Observation> &ostate, number t)
    {
        // std::cout << "OccupancyMDP::getRandomAction() " << std::endl;

        if (this->store_action_spaces_)
            return this->getActionSpaceAt(ostate->toState(), t)->sample()->toAction();
        else
            return this->computeRandomAction(ostate->toState()->toOccupancyState(), t);
    }

    std::shared_ptr<Action> OccupancyMDP::computeRandomAction(const std::shared_ptr<OccupancyStateInterface> &ostate, number t)
    {
        // std::cout << "OccupancyMDP::computeRandomAction() " << std::endl;

        // Vector for storing individual decision rules.
        std::vector<std::shared_ptr<DeterministicDecisionRule>> a;
        for (int agent = 0; agent < this->getUnderlyingProblem()->getNumAgents(); agent++)
        {
            // Input states for the a of agent.
            std::vector<std::shared_ptr<Item>> inputs;
            // Outputed actions for each of these.
            std::vector<std::shared_ptr<Item>> outputs;
            for (const auto& individual_history : ostate->getIndividualHistories(agent))
            {
                inputs.push_back(individual_history);
                outputs.push_back(std::static_pointer_cast<MultiDiscreteSpace>(this->getUnderlyingProblem()->getActionSpace(t))->get(agent)->sample());
            }
            a.push_back(std::make_shared<DeterministicDecisionRule>(inputs, outputs));
        }
        return std::make_shared<JointDeterministicDecisionRule>(a);
    }

    std::shared_ptr<Space> OccupancyMDP::computeActionSpaceAt(const std::shared_ptr<State> &ostate, number t)
    {
        // std::cout << "OccupancyMDP::computeActionSpaceAt() " << std::endl;

        // Vector of individual deterministic decision rules of each agent.
        std::vector<std::shared_ptr<Space>> individual_ddr_spaces;
        // For each agent from 0 to N-1:
        for (int agent = 0; agent < this->getUnderlyingProblem()->getNumAgents(); agent++)
        {
            // Get individual histories of agent i.
            std::set<std::shared_ptr<HistoryInterface>> individual_histories = ostate->toOccupancyState()->getIndividualHistories(agent);
            // Get individual history space of agent i.
            std::shared_ptr<Space> individual_history_space = std::make_shared<DiscreteSpace>(sdm::tools::set2vector(individual_histories));
            // Get action space of agent i.
            std::shared_ptr<Space> individual_action_space = std::static_pointer_cast<MultiDiscreteSpace>(this->getUnderlyingProblem()->getActionSpace(t))->get(agent);
            // Get individual ddr of agent i.
            std::shared_ptr<Space> individual_ddr_space = std::make_shared<FunctionSpace<DeterministicDecisionRule>>(individual_history_space, individual_action_space, this->store_action_spaces_);
            // Add it to the corresponding vector.
            individual_ddr_spaces.push_back(individual_ddr_space);
        }

        // Create the function space of joint deterministic decision rules.
        std::shared_ptr<Space> joint_ddr_space = std::make_shared<FunctionSpace<JointDeterministicDecisionRule>>(
            std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{nullptr}),
            std::make_shared<MultiDiscreteSpace>(individual_ddr_spaces, this->store_action_spaces_),
            this->store_action_spaces_);
        return joint_ddr_space;
    }

    Pair<std::shared_ptr<State>, double> OccupancyMDP::computeNextStateAndProbability(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        // std::cout << "OccupancyMDP::computeNextStateAndProbability() " << std::endl;

        return {this->computeNextState(ostate, action, observation, t), 1};
    }

    std::shared_ptr<State> OccupancyMDP::computeNextState(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        // std::cout << "OccupancyMDP::computeNextState() " << std::endl;

        // The new fully uncompressed occupancy state
        std::shared_ptr<State> fully_uncompressed_next_occupancy_state = std::make_shared<OccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());

        // The new one step left occupancy state
        std::shared_ptr<State> one_step_left_compressed_next_occupancy_state = std::make_shared<OccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());

        clock_t t_begin = clock();

        if (this->batch_size_ == 0)
        {
            // Compute exact next state
            std::tie(fully_uncompressed_next_occupancy_state, one_step_left_compressed_next_occupancy_state) = this->computeExactNextState(ostate, action, observation, t);
        }
        else
        {
            // Compute sampled next state
            std::tie(fully_uncompressed_next_occupancy_state, one_step_left_compressed_next_occupancy_state) = this->computeSampledNextState(ostate, action, observation, t);
        }
        OccupancyMDP::TIME_IN_NEXT_STATE += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);

        if (this->compression_)
        {
            t_begin = clock();

            // The new compressed occupancy state
            std::shared_ptr<State> compressed_next_occupancy_state;
            // Compress the occupancy state
            compressed_next_occupancy_state = one_step_left_compressed_next_occupancy_state->toOccupancyState()->compress();
            compressed_next_occupancy_state->toOccupancyState()->setFullyUncompressedOccupancy(fully_uncompressed_next_occupancy_state->toOccupancyState());
            compressed_next_occupancy_state->toOccupancyState()->setOneStepUncompressedOccupancy(one_step_left_compressed_next_occupancy_state->toOccupancyState());

            OccupancyMDP::TIME_IN_COMPRESS += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);

            return compressed_next_occupancy_state;
        }
        else
        {
            one_step_left_compressed_next_occupancy_state->toOccupancyState()->setFullyUncompressedOccupancy(fully_uncompressed_next_occupancy_state->toOccupancyState());
            one_step_left_compressed_next_occupancy_state->toOccupancyState()->setOneStepUncompressedOccupancy(one_step_left_compressed_next_occupancy_state->toOccupancyState());
            return one_step_left_compressed_next_occupancy_state;
        }
    }

    Pair<std::shared_ptr<State>, std::shared_ptr<State>> OccupancyMDP::computeExactNextState(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &, number t)
    {
        // std::cout << "OccupancyMDP::computeExactNextState() " << std::endl;

        auto occupancy_state = ostate->toOccupancyState();
        auto decision_rule = action->toDecisionRule();

        OccupancyMDP::PASSAGE_IN_NEXT_STATE++;
        OccupancyMDP::MEAN_SIZE_STATE += occupancy_state->getFullyUncompressedOccupancy()->getStates().size();

        // The new fully uncompressed occupancy state
        std::shared_ptr<OccupancyStateInterface> fully_uncompressed_next_occupancy_state = std::make_shared<OccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());
        // The new one step left occupancy state
        std::shared_ptr<OccupancyStateInterface> one_step_left_compressed_next_occupancy_state = std::make_shared<OccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());

        // For each joint history in the support of the fully uncompressed occupancy state
        for (const auto &joint_history : occupancy_state->getFullyUncompressedOccupancy()->getJointHistories())
        {
            // Apply the joint decision rule at joint_history to get the joint_action
            auto compressed_joint_history = occupancy_state->getCompressedJointHistory(joint_history);
            auto joint_action = this->applyDecisionRule(occupancy_state->toOccupancyState(), compressed_joint_history, decision_rule, t);

            // For each accessible belief at joint_history
            auto belief = occupancy_state->getFullyUncompressedOccupancy()->getBeliefAt(joint_history);

            // For each observation in
            for (auto &joint_observation : *this->getUnderlyingMPOMDP()->getObservationSpace(t))
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

                    auto next_compressed_joint_history = compressed_joint_history->expand(joint_observation->toObservation());
                    one_step_left_compressed_next_occupancy_state->addProbability(next_compressed_joint_history->toJointHistory(), next_belief->toBelief(), next_joint_history_probability);

                    // Update next history labels
                    one_step_left_compressed_next_occupancy_state->updateJointLabels(next_joint_history->toJointHistory()->getIndividualHistories(), next_compressed_joint_history->toJointHistory()->getIndividualHistories());
                }
            }
        }

        fully_uncompressed_next_occupancy_state->finalize();
        one_step_left_compressed_next_occupancy_state->finalize();

        return std::make_pair(fully_uncompressed_next_occupancy_state, one_step_left_compressed_next_occupancy_state);
    }

    Pair<std::shared_ptr<State>, std::shared_ptr<State>> OccupancyMDP::computeSampledNextState(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &, number t)
    {
        // std::cout << "OccupancyMDP::computeSampledNextState() " << std::endl;

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
        for (int k = 0; k < this->batch_size_; k++)
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

    std::shared_ptr<State> OccupancyMDP::nextState(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &decision_rule, number t, const std::shared_ptr<HSVI> &)
    {
        // Get the next state given a state and a decision rule
        return this->nextOccupancyState(ostate, decision_rule, nullptr, t);
    }

    std::shared_ptr<State> OccupancyMDP::nextOccupancyState(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        // std::cout << "OccupancyMDP::nextOccupancyState() " << std::endl;

        return BaseBeliefMDP<OccupancyState>::nextBelief(belief, action, observation, t);
    }

    std::shared_ptr<Action> OccupancyMDP::applyDecisionRule(const std::shared_ptr<OccupancyStateInterface> &ostate, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<Action> &decision_rule, number t) const
    {
        //
        auto individual_histories = joint_history->getIndividualHistories().toJoint<State>();
        // Get the selected action
        auto action = std::static_pointer_cast<Joint<std::shared_ptr<Action>>>(std::static_pointer_cast<JointDeterministicDecisionRule>(decision_rule)->act(individual_histories));
        // Get the adress of the action object from the space of available action object.
        return std::static_pointer_cast<MultiDiscreteSpace>(this->getUnderlyingProblem()->getActionSpace(t))->getItemAddress(*action->toJoint<Item>())->toAction();
    }

    std::shared_ptr<HistoryInterface> OccupancyMDP::getNextHistory(const std::shared_ptr<Observation> &observation)
    {
        if (this->batch_size_ == 0)
        {
            return this->current_state_->toOccupancyState()->getCompressedJointHistory(this->current_history_->expand(observation)->toJointHistory());
        }
        else
        {
            auto [jh, b] = this->current_state_->toOccupancyState()->sampleJointHistoryBelief();
            this->getUnderlyingProblem()->setInternalState(b->sampleState());
            return this->current_state_->toOccupancyState()->getCompressedJointHistory(jh);
        }
    }

    double OccupancyMDP::getReward(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t)
    {
        // std::cout << "OccupancyMDP::getReward() " << std::endl;

        clock_t t_begin = clock();

        auto state_action = std::make_pair(occupancy_state, decision_rule);
        auto successor = this->reward_graph_->getNode(0.0)->getSuccessor(state_action);
        double reward = 0;
        if (successor != nullptr)
        {
            // Return the successor node
            reward = successor->getData();
        }
        else
        {
            // For all histories in the occupancy state
            for (const auto &joint_history : occupancy_state->toOccupancyState()->getJointHistories())
            {
                // Get the belief corresponding to this history
                auto belief = occupancy_state->toOccupancyState()->getBeliefAt(joint_history);
                // Get the action from decision rule
                auto joint_action = this->applyDecisionRule(occupancy_state->toOccupancyState(), joint_history, decision_rule, t);
                // Update the expected reward
                reward += occupancy_state->toOccupancyState()->getProbability(joint_history) * this->getUnderlyingBeliefMDP()->getReward(belief, joint_action, t);
            }
            if (this->store_states_ && (this->store_action_spaces_ || this->store_actions_))
                this->reward_graph_->getNode(0.0)->addSuccessor(state_action, reward);
        }
        // FOR PROFILING
        OccupancyMDP::TIME_IN_GET_REWARD += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);
        return reward;
    }

    double OccupancyMDP::getRewardBelief(const std::shared_ptr<BeliefInterface> &state, const std::shared_ptr<Action> &action, number t)
    {
        return this->getUnderlyingBeliefMDP()->getReward(state, action, t);
    }

    double OccupancyMDP::getExpectedNextValue(const std::shared_ptr<ValueFunction> &value_function, const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &joint_decision_rule, number t)
    {
        // Check if we can skip the computation of the next occupancy state.
        // -> if the timestep is greater than the current horizon
        bool skip_compute_next_state = (value_function->isFiniteHorizon() && ((t + 1) >= value_function->getHorizon()));
        // Compute next state if required
        auto next_state = (skip_compute_next_state) ? nullptr : this->nextOccupancyState(occupancy_state, joint_decision_rule, nullptr, t);
        // Get value at the next state
        return value_function->getValueAt(next_state, t + 1);
    }

    std::shared_ptr<MPOMDPInterface> OccupancyMDP::getUnderlyingMPOMDP() const
    {
        return std::dynamic_pointer_cast<MPOMDPInterface>(this->getUnderlyingMDP());
    }

    std::shared_ptr<BeliefMDP> OccupancyMDP::getUnderlyingBeliefMDP() const
    {
        return this->belief_mdp_;
    }

    double OccupancyMDP::do_excess(double incumbent, double lb, double ub, double cost_so_far, double error, number horizon)
    {
        return std::min(ub - lb, cost_so_far + this->getUnderlyingProblem()->getDiscount(horizon) * ub - incumbent) - error / this->getWeightedDiscount(horizon);
    }

} // namespace sdm
