#include <sdm/world/occupancy_mdp.hpp>

namespace sdm
{

    template <class TOccupancyState>
    double BaseOccupancyMDP<TOccupancyState>::TIME_IN_NEXT_STATE = 0;

    template <class TOccupancyState>
    double BaseOccupancyMDP<TOccupancyState>::TIME_IN_COMPRESS = 0;

    template <class TOccupancyState>
    double BaseOccupancyMDP<TOccupancyState>::TIME_IN_GET_ACTION = 0;

    template <class TOccupancyState>
    double BaseOccupancyMDP<TOccupancyState>::TIME_IN_STEP = 0;

    template <class TOccupancyState>
    double BaseOccupancyMDP<TOccupancyState>::TIME_IN_UNDER_STEP = 0;

    template <class TOccupancyState>
    double BaseOccupancyMDP<TOccupancyState>::TIME_IN_GET_REWARD = 0;

    template <class TOccupancyState>
    double BaseOccupancyMDP<TOccupancyState>::TIME_IN_EXP_NEXT = 0;

    template <class TOccupancyState>
    double BaseOccupancyMDP<TOccupancyState>::TIME_IN_APPLY_DR = 0;

    template <class TOccupancyState>
    double BaseOccupancyMDP<TOccupancyState>::TIME_IN_NEXT_OSTATE = 0;

    template <class TOccupancyState>
    number BaseOccupancyMDP<TOccupancyState>::PASSAGE_IN_NEXT_STATE = 0;

    template <class TOccupancyState>
    unsigned long BaseOccupancyMDP<TOccupancyState>::MEAN_SIZE_STATE = 0;

    template <class TOccupancyState>
    BaseOccupancyMDP<TOccupancyState>::BaseOccupancyMDP()
    {
    }

    template <class TOccupancyState>
    BaseOccupancyMDP<TOccupancyState>::BaseOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &underlying_dpomdp, number memory, bool compression, bool store_states, bool store_actions, bool generate_action_spaces, int batch_size)
        : compression_(compression), generate_action_spaces_(generate_action_spaces), action_map_(std::make_shared<std::unordered_map<JointDeterministicDecisionRule, std::shared_ptr<Action>>>())
    {
        this->store_states_ = store_states;
        this->store_actions_ = store_actions;
        this->batch_size_ = batch_size;
        this->underlying_problem_ = underlying_dpomdp;

        // Initialize underlying belief mdp
        this->belief_mdp_ = std::make_shared<BeliefMDP>(underlying_dpomdp, batch_size);

        // Initialize initial history
        this->initial_history_ = std::make_shared<JointHistoryTree>(this->getUnderlyingMPOMDP()->getNumAgents(), (memory > 0) ? memory : -1);

        // Initialize initial occupancy state
        this->initial_state_ = std::make_shared<TOccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());

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

    template <class TOccupancyState>
    std::shared_ptr<Space> BaseOccupancyMDP<TOccupancyState>::getObservationSpace(number)
    {
        return std::make_shared<DiscreteSpace>(std::vector({sdm::NO_OBSERVATION}));
    }

    template <class TOccupancyState>
    bool BaseOccupancyMDP<TOccupancyState>::do_compression(number) const
    {
        return this->compression_;
    }

    template <class TOccupancyState>
    bool BaseOccupancyMDP<TOccupancyState>::checkCompatibility(const std::shared_ptr<Observation> &, const std::shared_ptr<Observation> &)
    {
        return true;
    }

    template <class TOccupancyState>
    std::shared_ptr<Space> BaseOccupancyMDP<TOccupancyState>::getActionSpaceAt(const std::shared_ptr<State> &ostate, number t)
    {
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
        BaseOccupancyMDP<TOccupancyState>::TIME_IN_GET_ACTION += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);
        return ostate->toOccupancyState()->getActionSpaceAt(t);
    }

    template <class TOccupancyState>
    std::shared_ptr<Space> BaseOccupancyMDP<TOccupancyState>::getActionSpaceAt(const std::shared_ptr<Observation> &ostate, number t)
    {
        return this->getActionSpaceAt(ostate->toState(), t);
    }

    template <class TOccupancyState>
    std::shared_ptr<Action> BaseOccupancyMDP<TOccupancyState>::getRandomAction(const std::shared_ptr<Observation> &ostate, number t)
    {
        if (this->generate_action_spaces_)
            return this->getActionSpaceAt(ostate->toState(), t)->sample()->toAction();
        else
            return this->computeRandomAction(ostate->toState()->toOccupancyState(), t);
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

    template <class TOccupancyState>
    std::shared_ptr<State> BaseOccupancyMDP<TOccupancyState>::nextOccupancyState(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, const std::shared_ptr<Observation> &observation, number t)
    {
        return BaseBeliefMDP<TOccupancyState>::nextBelief(occupancy_state, decision_rule, observation, t);
    }

    template <class TOccupancyState>
    Pair<std::shared_ptr<State>, double> BaseOccupancyMDP<TOccupancyState>::computeNextStateAndProbability(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        return {this->computeNextState(ostate, action, observation, t), 1.};
    }

    template <class TOccupancyState>
    std::shared_ptr<State> BaseOccupancyMDP<TOccupancyState>::computeNextState(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        clock_t t_begin = clock();
        // The new fully uncompressed occupancy state
        std::shared_ptr<State> next_fully_uncompressed_occupancy_state = std::make_shared<TOccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());

        // The new one step left occupancy state
        std::shared_ptr<State> next_one_step_left_compressed_occupancy_state = std::make_shared<TOccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());

        if (this->batch_size_ == 0)
        {
            // Compute exact next state
            std::tie(next_fully_uncompressed_occupancy_state, next_one_step_left_compressed_occupancy_state) = this->computeExactNextState(ostate, action, observation, t);
        }
        else
        {
            // Compute sampled next state
            std::tie(next_fully_uncompressed_occupancy_state, next_one_step_left_compressed_occupancy_state) = this->computeSampledNextState(ostate, action, observation, t);
        }
        BaseOccupancyMDP<TOccupancyState>::TIME_IN_NEXT_STATE += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);

        if (this->do_compression(t))
        {
            t_begin = clock();

            // The new compressed occupancy state
            std::shared_ptr<State> compressed_next_occupancy_state;
            // Compress the occupancy state
            compressed_next_occupancy_state = next_one_step_left_compressed_occupancy_state->toOccupancyState()->compress();
            compressed_next_occupancy_state->toOccupancyState()->setFullyUncompressedOccupancy(next_fully_uncompressed_occupancy_state->toOccupancyState());
            compressed_next_occupancy_state->toOccupancyState()->setOneStepUncompressedOccupancy(next_one_step_left_compressed_occupancy_state->toOccupancyState());

            BaseOccupancyMDP<TOccupancyState>::TIME_IN_COMPRESS += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);

            return compressed_next_occupancy_state;
        }
        else
        {
            next_one_step_left_compressed_occupancy_state->toOccupancyState()->setFullyUncompressedOccupancy(next_fully_uncompressed_occupancy_state->toOccupancyState());
            next_one_step_left_compressed_occupancy_state->toOccupancyState()->setOneStepUncompressedOccupancy(next_one_step_left_compressed_occupancy_state->toOccupancyState());
            return next_one_step_left_compressed_occupancy_state;
        }
    }

    template <class TOccupancyState>
    void BaseOccupancyMDP<TOccupancyState>::updateOccupancyStateProbability(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<BeliefInterface> &belief, double probability)
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
            if (this->getUnderlyingBeliefMDP()->state_space_.find(*aggregated_belief) == this->getUnderlyingBeliefMDP()->state_space_.end())
            {
                // Store the belief in the graph
                this->getUnderlyingBeliefMDP()->getMDPGraph()->addNode(aggregated_belief);
                // Store the belief in the belief space
                this->getUnderlyingBeliefMDP()->state_space_[*aggregated_belief] = aggregated_belief;
            }
            // Get the address of the belief
            auto ptr_belief = this->getUnderlyingBeliefMDP()->state_space_.at(*aggregated_belief);

            // Build fully uncompressed occupancy state
            occupancy_state->setProbability(joint_history->toJointHistory(), ptr_belief->toBelief(), occupancy_state->getProbability(joint_history) + probability);
        }
        else
        {
            // Build fully uncompressed occupancy state
            occupancy_state->setProbability(joint_history->toJointHistory(), belief->toBelief(), probability);
        }
    }

    template <class TOccupancyState>
    Pair<std::shared_ptr<State>, std::shared_ptr<State>> BaseOccupancyMDP<TOccupancyState>::computeExactNextState(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        auto compressed_occupancy_state = ostate->toOccupancyState();
        auto fully_uncompressed_occupancy_state = compressed_occupancy_state->getFullyUncompressedOccupancy();

        auto decision_rule = action->toDecisionRule();

        // The new fully uncompressed occupancy state
        std::shared_ptr<OccupancyStateInterface> next_fully_uncompressed_occupancy_state = std::make_shared<TOccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());
        // The new one step left occupancy state
        std::shared_ptr<OccupancyStateInterface> next_one_step_left_compressed_occupancy_state = std::make_shared<TOccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());
        try
        {
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
                for (const auto &joint_action : {jaction}) //decision_rule->getDistribution(compressed_joint_history)->getSupport())
                {
                    // Get p(u_t | o_t)
                    double proba_action = 1; //decision_rule->getProbability(compressed_joint_history, joint_action);
                    // For each observation in the space of joint observation
                    for (const auto &joint_observation : *this->getUnderlyingMPOMDP()->getObservationSpace(t))
                    {
                        if (this->checkCompatibility(joint_observation->toObservation(), observation))
                        {
                            // Get the next belief
                            std::shared_ptr<BeliefInterface> next_belief = this->getUnderlyingBeliefMDP()->nextBelief(belief, joint_action, joint_observation->toObservation(), t)->toBelief();

                            // Get p(z_{t+1} | b_t, u_t)
                            double proba_observation = this->getUnderlyingBeliefMDP()->getObservationProbability(belief, joint_action, next_belief->toBelief(), joint_observation->toObservation(), t);

                            // std::cout << "ph=" << proba_history << "/pa=" << proba_action << "po=" << proba_observation << std::endl;
                            // Compute the probability of next history, i.e. p(o') = p(o_t) * p(u_t | o_t) * p(z_{t+1} | b_t, u_t)
                            double next_joint_history_probability = proba_history * proba_action * proba_observation;

                            // If the next history probability is not zero
                            if (next_joint_history_probability > 0)
                            {
                                // Update new fully uncompressed occupancy state
                                std::shared_ptr<JointHistoryInterface> next_joint_history = joint_history->expand(/* joint_action, */ joint_observation->toObservation())->toJointHistory();
                                this->updateOccupancyStateProbability(next_fully_uncompressed_occupancy_state, next_joint_history, next_belief, next_joint_history_probability);

                                // Update new one step uncompressed occupancy state
                                std::shared_ptr<JointHistoryInterface> next_compressed_joint_history = compressed_joint_history->expand(/* joint_action, */ joint_observation->toObservation())->toJointHistory();
                                this->updateOccupancyStateProbability(next_one_step_left_compressed_occupancy_state, next_compressed_joint_history, next_belief, next_joint_history_probability);

                                // Update next history labels
                                next_one_step_left_compressed_occupancy_state->updateJointLabels(next_joint_history->toJointHistory()->getIndividualHistories(), next_compressed_joint_history->toJointHistory()->getIndividualHistories());
                            }
                        }
                    }
                }
            }

            // // Normalize the one step left occupancy state
            // double norm_one_step = next_one_step_left_compressed_occupancy_state->toBelief()->norm_1();
            // if (norm_one_step != 1.)
            // {
            //     next_one_step_left_compressed_occupancy_state->normalizeBelief(norm_one_step);
            // }

            // // Normalize the fully uncompressed occupancy state
            // double norm_fully = next_fully_uncompressed_occupancy_state->toBelief()->norm_1();
            // if (norm_fully != 1.)
            // {
            //     next_fully_uncompressed_occupancy_state->normalizeBelief(norm_fully);
            // }

            next_fully_uncompressed_occupancy_state->finalize();
            next_one_step_left_compressed_occupancy_state->finalize();
        }
        catch (const std::exception &exc)
        {
            // catch anything thrown within try block that derives from std::exception
            std::cerr << "OccupancyMDP::computeExactNextState(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }

        return std::make_pair(next_fully_uncompressed_occupancy_state, next_one_step_left_compressed_occupancy_state);
    }

    // template <class TOccupancyState>
    // Pair<std::shared_ptr<State>, std::shared_ptr<State>> BaseOccupancyMDP<TOccupancyState>::computeExactNextState(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &, number t)
    // {

    //     auto occupancy_state = ostate->toOccupancyState();
    //     auto decision_rule = action->toDecisionRule();

    //     BaseOccupancyMDP<TOccupancyState>::PASSAGE_IN_NEXT_STATE++;
    //     BaseOccupancyMDP<TOccupancyState>::MEAN_SIZE_STATE += occupancy_state->getFullyUncompressedOccupancy()->getStates().size();

    //     // The new fully uncompressed occupancy state
    //     std::shared_ptr<OccupancyStateInterface> next_fully_uncompressed_occupancy_state = std::make_shared<TOccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());
    //     // The new one step left occupancy state
    //     std::shared_ptr<OccupancyStateInterface> next_one_step_left_compressed_occupancy_state = std::make_shared<TOccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());

    //     // For each joint history in the support of the fully uncompressed occupancy state
    //     for (const auto &joint_history : occupancy_state->getFullyUncompressedOccupancy()->getJointHistories())
    //     {
    //         // Apply the joint decision rule at joint_history to get the joint_action
    //         auto compressed_joint_history = occupancy_state->getCompressedJointHistory(joint_history);
    //         auto joint_action = this->applyDecisionRule(occupancy_state->toOccupancyState(), compressed_joint_history, decision_rule, t);

    //         // For each accessible belief at joint_history
    //         auto belief = occupancy_state->getFullyUncompressedOccupancy()->getBeliefAt(joint_history);

    //         // For each observation in
    //         for (auto &joint_observation : *this->getUnderlyingMPOMDP()->getObservationSpace(t))
    //         {
    //             // Get the next joint history
    //             auto next_joint_history = joint_history->expand(joint_observation->toObservation());

    //             // Get the next belief
    //             auto next_belief = this->getUnderlyingBeliefMDP()->nextBelief(belief, joint_action, joint_observation->toObservation(), t);

    //             // Compute the probability of next history, i.e. p(o') = p(o) * p(z | b, u)
    //             double next_joint_history_probability = occupancy_state->getFullyUncompressedOccupancy()->getProbability(joint_history) * this->getUnderlyingBeliefMDP()->getObservationProbability(belief, joint_action, next_belief->toBelief(), joint_observation->toObservation(), t);

    //             // If the next history probability is not zero
    //             if (next_joint_history_probability > 0)
    //             {
    //                 // Build fully uncompressed occupancy state
    //                 next_fully_uncompressed_occupancy_state->addProbability(next_joint_history->toJointHistory(), next_belief->toBelief(), next_joint_history_probability);

    //                 // Update the probability of being in this next history (for the one step left uncompressed occupancy state)

    //                 auto next_compressed_joint_history = compressed_joint_history->expand(joint_observation->toObservation());
    //                 next_one_step_left_compressed_occupancy_state->addProbability(next_compressed_joint_history->toJointHistory(), next_belief->toBelief(), next_joint_history_probability);

    //                 // Update next history labels
    //                 next_one_step_left_compressed_occupancy_state->updateJointLabels(next_joint_history->toJointHistory()->getIndividualHistories(), next_compressed_joint_history->toJointHistory()->getIndividualHistories());
    //             }
    //         }
    //     }

    //     next_fully_uncompressed_occupancy_state->finalize();
    //     next_one_step_left_compressed_occupancy_state->finalize();

    //     return std::make_pair(next_fully_uncompressed_occupancy_state, next_one_step_left_compressed_occupancy_state);
    // }

    template <class TOccupancyState>
    Pair<std::shared_ptr<State>, std::shared_ptr<State>> BaseOccupancyMDP<TOccupancyState>::computeSampledNextState(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {

        // The new fully uncompressed occupancy state
        std::shared_ptr<OccupancyStateInterface> next_fully_uncompressed_occupancy_state = std::make_shared<TOccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());

        // The new one step left occupancy state
        std::shared_ptr<OccupancyStateInterface> next_one_step_left_compressed_occupancy_state = std::make_shared<TOccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());

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

        // Finalize the one step left compressed occupancy state
        next_one_step_left_compressed_occupancy_state->finalize();
        next_fully_uncompressed_occupancy_state->finalize();

        return std::make_pair(next_fully_uncompressed_occupancy_state, next_one_step_left_compressed_occupancy_state);
    }

    template <class TOccupancyState>
    std::shared_ptr<Action> BaseOccupancyMDP<TOccupancyState>::applyDecisionRule(const std::shared_ptr<OccupancyStateInterface> &, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<Action> &decision_rule, number t) const
    {
        clock_t t_begin = clock();
        // Get the selected joint action
        auto action = std::static_pointer_cast<JointDeterministicDecisionRule>(decision_rule)->act(joint_history->getIndividualHistories().toJoint<State>());

        // Transform the selected joint action into joint action address
        auto joint_action = std::static_pointer_cast<Joint<std::shared_ptr<Action>>>(action);

        // Get the adress of the joint action object from the space of available joint action object.
        auto joint_action_address = std::static_pointer_cast<MultiDiscreteSpace>(this->getUnderlyingProblem()->getActionSpace(t))->getItemAddress(*joint_action->toJoint<Item>());
        BaseOccupancyMDP<TOccupancyState>::TIME_IN_APPLY_DR += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);
        return joint_action_address->toAction();
    }

    template <class TOccupancyState>
    std::shared_ptr<HistoryInterface> BaseOccupancyMDP<TOccupancyState>::getNextHistory(const std::shared_ptr<Observation> &observation)
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

    template <class TOccupancyState>
    double BaseOccupancyMDP<TOccupancyState>::getReward(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t)
    {

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
            if (this->store_states_ && this->store_actions_)
                this->reward_graph_->getNode(0.0)->addSuccessor(state_action, reward);
        }

        // FOR PROFILING
        BaseOccupancyMDP<TOccupancyState>::TIME_IN_GET_REWARD += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);
        return reward;
    }

    template <class TOccupancyState>
    double BaseOccupancyMDP<TOccupancyState>::getRewardBelief(const std::shared_ptr<BeliefInterface> &state, const std::shared_ptr<Action> &action, number t)
    {
        return this->getUnderlyingBeliefMDP()->getReward(state, action, t);
    }

    template <class TOccupancyState>
    std::shared_ptr<MPOMDPInterface> BaseOccupancyMDP<TOccupancyState>::getUnderlyingMPOMDP() const
    {
        return std::dynamic_pointer_cast<MPOMDPInterface>(this->getUnderlyingMDP());
    }

    template <class TOccupancyState>
    std::shared_ptr<BeliefMDP> BaseOccupancyMDP<TOccupancyState>::getUnderlyingBeliefMDP() const
    {
        return this->belief_mdp_;
    }

    template <class TOccupancyState>
    double BaseOccupancyMDP<TOccupancyState>::do_excess(double incumbent, double lb, double ub, double cost_so_far, double error, number horizon)
    {
        return std::min(ub - lb, cost_so_far + this->getUnderlyingProblem()->getDiscount(horizon) * ub - incumbent) - error / this->getWeightedDiscount(horizon);
    }

    // -------------------
    //     RL METHODS
    // -------------------

    template <class TOccupancyState>
    std::shared_ptr<Observation> BaseOccupancyMDP<TOccupancyState>::reset()
    {
        this->current_history_ = this->initial_history_;
        return BaseBeliefMDP<TOccupancyState>::reset();
    }

    template <class TOccupancyState>
    std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> BaseOccupancyMDP<TOccupancyState>::step(std::shared_ptr<Action> action)
    {
        clock_t t_begin = clock(), t_tmp = clock();
        auto joint_action = this->applyDecisionRule(this->current_state_->toOccupancyState(), this->current_history_->toJointHistory(), action, this->step_);
        OccupancyMDP::TIME_IN_APPLY_DR += ((float)(clock() - t_tmp) / CLOCKS_PER_SEC);

        t_tmp = clock();
        auto [observation, mpomdp_reward, __] = this->getUnderlyingProblem()->step(joint_action);
        OccupancyMDP::TIME_IN_UNDER_STEP += ((float)(clock() - t_tmp) / CLOCKS_PER_SEC);

        double occupancy_reward = this->getReward(this->current_state_, action, this->step_);

        t_tmp = clock();
        this->current_state_ = this->nextBelief(this->current_state_, action, sdm::NO_OBSERVATION, this->step_);
        BaseOccupancyMDP<TOccupancyState>::TIME_IN_NEXT_OSTATE += ((float)(clock() - t_tmp) / CLOCKS_PER_SEC);

        this->current_history_ = this->getNextHistory(observation);
        this->step_++;

        BaseOccupancyMDP<TOccupancyState>::TIME_IN_STEP += ((float)(clock() - t_begin) / CLOCKS_PER_SEC);
        return std::make_tuple(this->current_state_, std::vector<double>{occupancy_reward, mpomdp_reward[0]}, (this->step_ > this->getUnderlyingMPOMDP()->getHorizon()));
    }

    // template <class TOccupancyState>
    // std::shared_ptr<Action> BaseOccupancyMDP<TOccupancyState>::getActionPointer(std::shared_ptr<Action> action_tmp)
    // {
    //     if (this->store_actions_)
    //     {
    //         if (this->action_map_->find(*action_tmp->toJointDeterministicDecisionRule()) == this->action_map_->end())
    //         {
    //             this->action_map_->emplace(*action_tmp->toJointDeterministicDecisionRule(), action_tmp);
    //         }
    //         return this->action_map_->at(*action_tmp->toJointDeterministicDecisionRule());
    //     }
    //     return action_tmp;
    // }

} // namespace sdm
