#include <memory>
#include <sdm/world/occupancy_mdp.hpp>

namespace sdm
{

    template <class TOccupancyState>
    BaseOccupancyMDP<TOccupancyState>::BaseOccupancyMDP()
    {
    }

    template <class TOccupancyState>
    BaseOccupancyMDP<TOccupancyState>::~BaseOccupancyMDP()
    {
    }

    template <class TOccupancyState>
    BaseOccupancyMDP<TOccupancyState>::BaseOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &underlying_dpomdp, number memory, bool store_states, bool store_actions, int batch_size)
        : underlying_mpomdp(underlying_dpomdp), memory(memory)
    {
        this->store_states_ = store_states;
        this->store_actions_ = store_actions;
        this->batch_size_ = batch_size;
        this->underlying_problem_ = underlying_dpomdp;

        // Initialize underlying belief mdp
        this->belief_mdp_ = std::make_shared<BeliefMDP>(underlying_dpomdp, batch_size);

        // Initialize initial history
        this->initial_history_ = std::make_shared<JointHistoryTree>(this->getUnderlyingMDP()->getNumAgents(), (this->memory > 0) ? this->memory : -1);

        // Initialize initial occupancy state
        this->initial_state_ = std::make_shared<TOccupancyState>(this->getUnderlyingMDP()->getNumAgents());

        this->initial_state_->toOccupancyState()->setProbability(this->initial_history_->toJointHistory(), this->belief_mdp_->getInitialState()->toBelief(), 1);
        this->initial_state_->toOccupancyState()->finalize();

        // Initialize Transition Graph
        this->mdp_graph_ = std::make_shared<Graph<std::shared_ptr<State>, Pair<std::shared_ptr<Action>, std::shared_ptr<Observation>>>>();
        this->mdp_graph_->addNode(this->initial_state_);

        this->reward_graph_ = std::make_shared<Graph<double, Pair<std::shared_ptr<State>, std::shared_ptr<Action>>>>();
        this->reward_graph_->addNode(0.0);
    }

    template <class TOccupancyState>
    BaseOccupancyMDP<TOccupancyState>::BaseOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &underlying_dpomdp, Config config)
        : BaseOccupancyMDP<TOccupancyState>(underlying_dpomdp,
                                            config.get("memory", 0),
                                            config.get("store_states", true),
                                            config.get("store_actions", true),
                                            config.get("batch_size", 0))
    {
        auto opt_int = config.getOpt<int>("state_type");
        auto opt_str = config.getOpt("state_type");
        if (opt_int.has_value())
        {
            this->setStateType((StateType)opt_int.value());
        }
        else if (opt_str.has_value())
        {
            auto iter = STATE_TYPE_MAP.find(opt_str.value());
            this->setStateType((iter != STATE_TYPE_MAP.end()) ? iter->second : StateType::COMPRESSED);
        }
    }

    // template <class TOccupancyState>
    // BaseOccupancyMDP<TOccupancyState>::BaseOccupancyMDP(Config config)
    // {
    // }

    template <class TOccupancyState>
    std::shared_ptr<Space> BaseOccupancyMDP<TOccupancyState>::getObservationSpaceAt(const std::shared_ptr<State> &, const std::shared_ptr<Action> &, number)
    {
        return std::make_shared<DiscreteSpace>(std::vector({sdm::NO_OBSERVATION}));
    }

    template <class TOccupancyState>
    void BaseOccupancyMDP<TOccupancyState>::setStateType(const StateType &state_type)
    {
        this->state_type = state_type;
    }

    template <class TOccupancyState>
    bool BaseOccupancyMDP<TOccupancyState>::checkCompatibility(const std::shared_ptr<Observation> &, const std::shared_ptr<Observation> &)
    {
        return true;
    }

    template <class TOccupancyState>
    std::shared_ptr<MPOMDPInterface> BaseOccupancyMDP<TOccupancyState>::getUnderlyingMPOMDP() const
    {
        return underlying_mpomdp;
    }

    template <class TOccupancyState>
    std::shared_ptr<BeliefMDPInterface> BaseOccupancyMDP<TOccupancyState>::getUnderlyingBeliefMDP()
    {
        return this->belief_mdp_;
    }

    template <class TOccupancyState>
    double BaseOccupancyMDP<TOccupancyState>::do_excess(double incumbent, double lb, double ub, double cost_so_far, double error, number horizon)
    {
        // return std::min(ub - lb, cost_so_far + this->getUnderlyingProblem()->getDiscount(horizon) * ub - incumbent) - error / this->getWeightedDiscount(horizon);
        return (ub - lb) - error / this->getWeightedDiscount(horizon);
    }

    // -------------------
    //     RL METHODS
    // -------------------

    template <class TOccupancyState>
    std::shared_ptr<State> BaseOccupancyMDP<TOccupancyState>::reset()
    {
        return BaseBeliefMDP<TOccupancyState>::reset();
    }

    template <class TOccupancyState>
    std::tuple<std::shared_ptr<State>, std::vector<double>, bool> BaseOccupancyMDP<TOccupancyState>::step(std::shared_ptr<Action> action)
    {
        // Compute next reward
        double occupancy_reward = this->getReward(this->current_state_, action, this->step_);

        // Compute next occupancy state
        this->current_state_ = this->getNextStateAndProba(this->current_state_, action, sdm::NO_OBSERVATION, this->step_).first;

        // Increment step
        this->step_++;

        return std::make_tuple(this->current_state_, std::vector<double>(this->getUnderlyingMDP()->getNumAgents(), occupancy_reward), (this->step_ > this->getUnderlyingMPOMDP()->getHorizon()));
    }

    // -----------------------
    // Manipulate actions
    // -------------------------

    template <class TOccupancyState>
    std::shared_ptr<Space> BaseOccupancyMDP<TOccupancyState>::getActionSpaceAt(const std::shared_ptr<State> &ostate, number t)
    {
        auto occupancy_state = ostate->toOccupancyState();
        // If the action space corresponding to this ostate and t does not exist:
        if (occupancy_state->getActionSpaceAt(t) == nullptr)
        {
            // Compute the action space at this occupancy state and timestep
            std::shared_ptr<Space> joint_ddr_space = this->computeActionSpaceAt(ostate, t);

            if (!this->store_actions_)
            {
                return joint_ddr_space;
            }

            // Store the action space for state o
            occupancy_state->setActionSpaceAt(t, joint_ddr_space);
        }
        // Return the action space corresponding to this ostate and t.
        return occupancy_state->getActionSpaceAt(t);
    }

    template <class TOccupancyState>
    std::shared_ptr<Action> BaseOccupancyMDP<TOccupancyState>::getRandomAction(const std::shared_ptr<State> &ostate, number t)
    {
        if (this->store_actions_)
            return this->getActionSpaceAt(ostate, t)->sample()->toAction();
        else
            return this->computeRandomAction(ostate->toOccupancyState(), t);
    }

    template <class TOccupancyState>
    std::shared_ptr<Action> BaseOccupancyMDP<TOccupancyState>::computeRandomAction(const std::shared_ptr<OccupancyStateInterface> &ostate, number t)
    {
        // Vector for storing individual decision rules.
        std::vector<std::shared_ptr<DeterministicDecisionRule>> a;
        for (int agent = 0; agent < this->getUnderlyingProblem()->getNumAgents(); agent++)
        {
            // Input states for the a of agent.
            std::vector<std::shared_ptr<Item>> inputs;
            // Outputed actions for each of these.
            std::vector<std::shared_ptr<Item>> outputs;
            for (const auto &individual_history : ostate->getIndividualHistories(agent))
            {
                inputs.push_back(individual_history);
                outputs.push_back(this->getUnderlyingMPOMDP()->getActionSpace(agent, t)->sample());
            }
            a.push_back(std::make_shared<DeterministicDecisionRule>(inputs, outputs));
        }
        return std::make_shared<JointDeterministicDecisionRule>(a);
    }

    template <class TOccupancyState>
    std::shared_ptr<Space> BaseOccupancyMDP<TOccupancyState>::computeActionSpaceAt(const std::shared_ptr<State> &ostate, number t)
    {
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
            std::shared_ptr<Space> individual_action_space = this->getUnderlyingMPOMDP()->getActionSpace(agent, t);
            // Get individual ddr of agent i.
            std::shared_ptr<Space> individual_ddr_space = std::make_shared<FunctionSpace<DeterministicDecisionRule>>(individual_history_space, individual_action_space, this->store_actions_);
            // Add it to the corresponding vector.
            individual_ddr_spaces.push_back(individual_ddr_space);
        }

        // Create the function space of joint deterministic decision rules.
        std::shared_ptr<Space> joint_ddr_space = std::make_shared<FunctionSpace<JointDeterministicDecisionRule>>(
            std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{nullptr}),
            std::make_shared<MultiDiscreteSpace>(individual_ddr_spaces, this->store_actions_),
            this->store_actions_);

        return joint_ddr_space;
    }

    // -----------------------
    // Manipulate states
    // -------------------------

    template <class TOccupancyState>
    Pair<std::shared_ptr<State>, double> BaseOccupancyMDP<TOccupancyState>::computeNextStateAndProbability(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        if (this->batch_size_ == 0)
        {
            // Compute exact next state
            return this->computeExactNextState(ostate, action, observation, t);
        }
        else
        {
            // Compute sampled next state
            return this->computeSampledNextState(ostate, action, observation, t);
        }
    }

    template <class TOccupancyState>
    Pair<std::shared_ptr<State>, double> BaseOccupancyMDP<TOccupancyState>::computeExactNextState(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        auto compressed_occupancy_state = ostate->toOccupancyState();
        auto fully_uncompressed_occupancy_state = compressed_occupancy_state->getFullyUncompressedOccupancy();

        auto decision_rule = action->toDecisionRule();

        // The new fully uncompressed occupancy state
        std::shared_ptr<OccupancyStateInterface> next_fully_uncompressed_occupancy_state = std::make_shared<TOccupancyState>(this->getUnderlyingMDP()->getNumAgents());
        // The new one step left occupancy state
        std::shared_ptr<OccupancyStateInterface> next_one_step_left_compressed_occupancy_state = std::make_shared<TOccupancyState>(this->getUnderlyingMDP()->getNumAgents());

        // For each joint history in the support of the fully uncompressed occupancy state
        for (const auto &joint_history : fully_uncompressed_occupancy_state->getJointHistories())
        {
            // Get p(o_t)
            double proba_history = fully_uncompressed_occupancy_state->getProbability(joint_history);
            // Get compressed joint history
            auto compressed_joint_history = compressed_occupancy_state->getCompressedJointHistory(joint_history);

            // Apply decision rule and get action
            auto jaction = this->applyDecisionRule(compressed_occupancy_state, compressed_joint_history, decision_rule, t);
            // Get the corresponding belief
            std::shared_ptr<BeliefInterface> belief = fully_uncompressed_occupancy_state->getBeliefAt(joint_history);

            // For each action that is likely to be taken
            for (const auto &joint_action : {jaction}) // decision_rule->getDistribution(compressed_joint_history)->getSupport())
            {
                // Get p(u_t | o_t)
                double proba_action = 1; // decision_rule->getProbability(compressed_joint_history, joint_action);

                // For each observation in the space of joint observation
                for (auto jobs : *this->getUnderlyingMPOMDP()->getObservationSpace(t))
                {
                    auto joint_observation = jobs->toObservation();
                    if (this->checkCompatibility(joint_observation, observation))
                    {
                        // Get the next belief and p(z_{t+1} | b_t, u_t)
                        auto [next_belief, proba_observation] = this->getUnderlyingBeliefMDP()->getNextStateAndProba(belief, joint_action, joint_observation, t);

                        double next_joint_history_probability = proba_history * proba_action * proba_observation;

                        // If the next history probability is not zero
                        if (next_joint_history_probability > 0)
                        {
                            std::shared_ptr<JointHistoryInterface> next_joint_history = joint_history->expand(joint_observation /*, joint_action*/)->toJointHistory();

                            // Update new fully uncompressed occupancy state
                            this->updateOccupancyStateProba(next_fully_uncompressed_occupancy_state, next_joint_history, next_belief->toBelief(), next_joint_history_probability);

                            // Update new one step uncompressed occupancy state
                            std::shared_ptr<JointHistoryInterface> next_compressed_joint_history = compressed_joint_history->expand(joint_observation /*, joint_action*/)->toJointHistory();
                            this->updateOccupancyStateProba(next_one_step_left_compressed_occupancy_state, next_compressed_joint_history, next_belief->toBelief(), next_joint_history_probability);

                            // Update next history labels
                            next_one_step_left_compressed_occupancy_state->updateJointLabels(next_joint_history->getIndividualHistories(), next_compressed_joint_history->getIndividualHistories());
                        }
                    }
                }
            }
        }

        return this->finalizeNextState(next_one_step_left_compressed_occupancy_state, next_fully_uncompressed_occupancy_state, t);
    }

    template <class TOccupancyState>
    Pair<std::shared_ptr<State>, double> BaseOccupancyMDP<TOccupancyState>::computeSampledNextState(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        // The new fully uncompressed occupancy state
        std::shared_ptr<OccupancyStateInterface> next_fully_uncompressed_occupancy_state = std::make_shared<TOccupancyState>(this->getUnderlyingMDP()->getNumAgents());

        // The new one step left occupancy state
        std::shared_ptr<OccupancyStateInterface> next_one_step_left_compressed_occupancy_state = std::make_shared<TOccupancyState>(this->getUnderlyingMDP()->getNumAgents());

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
            auto [next_joint_observation, _, __] = this->getUnderlyingProblem()->step(joint_action, false);
            if (this->checkCompatibility(next_joint_observation, observation))
            {
                // Sample next state.
                auto next_state = this->getUnderlyingProblem()->getInternalState();
                // Sample next joint history.
                auto next_joint_history = joint_history->expand(next_joint_observation);
                // If next_joint_history is seen for the first time:
                if (w_y_bag.find(next_joint_history) == w_y_bag.end())
                {
                    // Set next_joint_history's counter to 1.
                    w_y_bag[next_joint_history].first = 1;
                    // No need to check if (next_joint_history, next_state) was seen before, the answer is no.
                    // Set (next_joint_history, next_state)'s counter to 1.
                    w_y_bag[next_joint_history].second[next_state] = 1;
                    // These two will be needed later.
                    w__o_b_u_z__map[next_joint_history] = std::make_tuple(joint_history, belief, joint_action, next_joint_observation);
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
            next_fully_uncompressed_occupancy_state->addProbability(next_joint_history->toJointHistory(), next_belief, next_joint_history_count / this->batch_size_);
            auto joint_history_ = next_joint_history->getPreviousHistory();
            auto observation_ = next_joint_history->getLastObservation();
            auto compressed_joint_history = ostate->toOccupancyState()->getCompressedJointHistory(joint_history->toJointHistory());
            auto next_compressed_joint_history = compressed_joint_history->expand(observation->toObservation());
            next_one_step_left_compressed_occupancy_state->addProbability(next_compressed_joint_history->toJointHistory(), next_belief->toBelief(), next_joint_history_count / this->batch_size_);
            next_one_step_left_compressed_occupancy_state->updateJointLabels(next_joint_history->toJointHistory()->getIndividualHistories(), next_compressed_joint_history->toJointHistory()->getIndividualHistories());
        }
        //
        this->getUnderlyingProblem()->setInternalState(true_state);

        return this->finalizeNextState(next_one_step_left_compressed_occupancy_state, next_fully_uncompressed_occupancy_state, t);
    }

    template <class TOccupancyState>
    std::shared_ptr<Action> BaseOccupancyMDP<TOccupancyState>::applyDecisionRule(const std::shared_ptr<OccupancyStateInterface> &, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<Action> &decision_rule, number t) const
    {
        // Get the selected joint action
        auto action = std::static_pointer_cast<JointDeterministicDecisionRule>(decision_rule)->act(joint_history->getIndividualHistories().toJoint<State>());

        // Transform the selected joint action into joint action address
        auto joint_action = std::static_pointer_cast<Joint<std::shared_ptr<Action>>>(action);

        // Get the adress of the joint action object from the space of available joint action object.
        auto joint_action_address = std::static_pointer_cast<MultiDiscreteSpace>(this->getUnderlyingMPOMDP()->getActionSpace(t))->getItemAddress(*joint_action->toJoint<Item>());
        return joint_action_address->toAction();
    }

    template <class TOccupancyState>
    double BaseOccupancyMDP<TOccupancyState>::getReward(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &decision_rule, number t)
    {
        auto state_action = std::make_pair(ostate, decision_rule);
        auto successor = this->reward_graph_->getSuccessor(0.0, state_action);
        double reward = 0.0;
        if (successor != nullptr)
        {
            // Return the successor node
            reward = successor->getData();
        }
        else
        {
            auto occupancy_state = ostate->toOccupancyState();
            // For all histories in the occupancy state
            for (const auto &joint_history : occupancy_state->getJointHistories())
            {
                // Get the belief corresponding to this history
                auto belief = occupancy_state->getBeliefAt(joint_history);
                // Get the action from decision rule
                auto joint_action = this->applyDecisionRule(occupancy_state, joint_history, decision_rule, t);
                // Update the expected reward
                reward += occupancy_state->getProbability(joint_history) * this->getUnderlyingBeliefMDP()->getReward(belief, joint_action, t);
            }
            if (this->store_states_ && this->store_actions_)
                this->reward_graph_->addSuccessor(0.0, state_action, reward);
        }

        return reward;
    }

    template <class TOccupancyState>
    double BaseOccupancyMDP<TOccupancyState>::getRewardBelief(const std::shared_ptr<BeliefInterface> &state, const std::shared_ptr<Action> &action, number t)
    {
        return this->getUnderlyingBeliefMDP()->getReward(state, action, t);
    }

    template <class TOccupancyState>
    Pair<std::shared_ptr<OccupancyStateInterface>, double> BaseOccupancyMDP<TOccupancyState>::finalizeNextState(const std::shared_ptr<OccupancyStateInterface> &one_step_occupancy_state, const std::shared_ptr<OccupancyStateInterface> &fully_uncompressed_occupancy_state, number t)
    {
        // // Finalize and normalize the one step left occupancy state
        // next_one_step_left_compressed_occupancy_state->finalize();
        // double norm_one_step = next_one_step_left_compressed_occupancy_state->norm_1();
        // next_one_step_left_compressed_occupancy_state->normalizeBelief(norm_one_step);

        // // Finalize and normalize the fully uncompressed occupancy state
        // next_fully_uncompressed_occupancy_state->finalize(false);
        // double norm_fully = next_fully_uncompressed_occupancy_state->norm_1();
        // next_fully_uncompressed_occupancy_state->normalizeBelief(norm_fully);

        // if (this->doCompression(t))
        // {
        //     // The new compressed occupancy state
        //     std::shared_ptr<OccupancyStateInterface> next_compressed_occupancy_state;
        //     // Compress the occupancy state
        //     next_compressed_occupancy_state = next_one_step_left_compressed_occupancy_state->compress();

        //     double norm_compressed = next_compressed_occupancy_state->norm_1();
        //     next_compressed_occupancy_state->normalizeBelief(norm_compressed);
        //     // Set
        //     next_compressed_occupancy_state->setFullyUncompressedOccupancy(next_fully_uncompressed_occupancy_state);
        //     next_compressed_occupancy_state->setOneStepUncompressedOccupancy(next_one_step_left_compressed_occupancy_state);

        //     return {next_compressed_occupancy_state, norm_one_step};
        // }
        // else
        // {
        //     next_one_step_left_compressed_occupancy_state->setFullyUncompressedOccupancy(next_fully_uncompressed_occupancy_state);
        //     next_one_step_left_compressed_occupancy_state->setOneStepUncompressedOccupancy(next_one_step_left_compressed_occupancy_state);
        //     return {next_one_step_left_compressed_occupancy_state, norm_one_step};
        // }

        // Finalize and normalize the one step left occupancy state
        one_step_occupancy_state->finalize();
        double norm_one_step = one_step_occupancy_state->norm_1();
        one_step_occupancy_state->normalizeBelief(norm_one_step);

        // Finalize and normalize the fully uncompressed occupancy state
        fully_uncompressed_occupancy_state->finalize(false);
        double norm_fully = fully_uncompressed_occupancy_state->norm_1();
        fully_uncompressed_occupancy_state->normalizeBelief(norm_fully);

        if (this->state_type == StateType::FULLY_UNCOMPRESSED)
        {
            one_step_occupancy_state->setFullyUncompressedOccupancy(fully_uncompressed_occupancy_state);
            one_step_occupancy_state->setOneStepUncompressedOccupancy(one_step_occupancy_state);
            return {one_step_occupancy_state, norm_one_step};
        }
        else
        {
            // The new compressed occupancy state
            std::shared_ptr<OccupancyStateInterface> compressed_occupancy_state;

            // Compress the occupancy state
            compressed_occupancy_state = one_step_occupancy_state->compress();

            double norm_compressed = compressed_occupancy_state->norm_1();
            compressed_occupancy_state->normalizeBelief(norm_compressed);

            if (this->state_type == StateType::COMPRESSED)
            {
                compressed_occupancy_state->setFullyUncompressedOccupancy(fully_uncompressed_occupancy_state);
                compressed_occupancy_state->setOneStepUncompressedOccupancy(one_step_occupancy_state);
                one_step_occupancy_state->setCompressedOccupancy(compressed_occupancy_state);
                return {compressed_occupancy_state, norm_one_step};
            }
            else if (this->state_type == StateType::ONE_STEP_UNCOMPRESSED)
            {
                one_step_occupancy_state->setFullyUncompressedOccupancy(fully_uncompressed_occupancy_state);
                one_step_occupancy_state->setCompressedOccupancy(compressed_occupancy_state);
                compressed_occupancy_state->setOneStepUncompressedOccupancy(one_step_occupancy_state);
                compressed_occupancy_state->setFullyUncompressedOccupancy(fully_uncompressed_occupancy_state);
                return {one_step_occupancy_state, norm_one_step};
            }
            else
            {
                throw sdm::exception::TypeError("StateType not recognized.");
            }
        }
    }

    template <class TOccupancyState>
    void BaseOccupancyMDP<TOccupancyState>::updateOccupancyStateProba(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<BeliefInterface> &belief, double probability)
    {

        if (occupancy_state->getProbability(joint_history) > 0.)
        {
            // Get the probability of being in each belief
            double proba_belief1 = occupancy_state->getProbability(joint_history), proba_belief2 = probability;
            // Cast to belief structure
            std::shared_ptr<Belief> belief1 = std::dynamic_pointer_cast<Belief>(occupancy_state->getBeliefAt(joint_history)), belief2 = std::dynamic_pointer_cast<Belief>(belief);

            // Aggregate beliefs
            std::shared_ptr<Belief> aggregated_belief = belief1->add(belief2, proba_belief1, proba_belief2);

            // Normalize the resulting belief
            aggregated_belief->normalizeBelief(aggregated_belief->norm_1());

            // Check if the belief already exists in the belief space
            if (this->belief_mdp_->state_space_.find(*aggregated_belief) == this->belief_mdp_->state_space_.end())
            {
                // Store the belief in the graph
                this->belief_mdp_->getMDPGraph()->addNode(aggregated_belief);
                // Store the belief in the belief space
                this->belief_mdp_->state_space_[*aggregated_belief] = aggregated_belief;
            }
            // Get the address of the belief
            auto ptr_belief = this->belief_mdp_->state_space_.at(*aggregated_belief);

            // Build fully uncompressed occupancy state
            occupancy_state->setProbability(joint_history, ptr_belief->toBelief(), occupancy_state->getProbability(joint_history) + probability);
        }
        else
        {
            // Build fully uncompressed occupancy state
            occupancy_state->setProbability(joint_history, belief, probability);
        }
    }

    template <class TOccupancyState>
    std::shared_ptr<BaseOccupancyMDP<TOccupancyState>> BaseOccupancyMDP<TOccupancyState>::getptr()
    {
        return std::enable_shared_from_this<BaseOccupancyMDP<TOccupancyState>>::shared_from_this();
    }

} // namespace sdm
