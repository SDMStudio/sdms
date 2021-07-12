#include <sdm/world/occupancy_mdp.hpp>

namespace sdm
{

    OccupancyMDP::OccupancyMDP()
    {
    }

    OccupancyMDP::OccupancyMDP(const std::shared_ptr<MPOMDPInterface> &underlying_dpomdp, number memory, bool compression, bool store_states, bool store_actions, int batch_size)
        : compression_(compression), store_actions_(store_actions)
    {
        this->store_states_ = store_states;
        this->batch_size_ = batch_size;

        // Set underlying problem
        this->underlying_problem = underlying_dpomdp;

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
    }

    std::shared_ptr<Observation> OccupancyMDP::reset()
    {
        this->current_history_ = this->initial_history_;
        return BaseBeliefMDP<OccupancyState>::reset();
    }

    std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> OccupancyMDP::step(std::shared_ptr<Action> action)
    {
        auto joint_action = this->applyDecisionRule(this->current_state_->toOccupancyState(), this->current_history_->toJointHistory(), action, this->step_);
        auto [observation, rewards, is_done] = this->getUnderlyingProblem()->step(joint_action);
        double occupancy_reward = this->getReward(this->current_state_, action, this->step_);
        this->current_state_ = this->nextOccupancyState(this->current_state_, action, nullptr, this->step_);
        this->current_history_ = this->getNextHistory(observation);
        this->step_++;
        return std::make_tuple(this->current_state_, std::vector<double>{occupancy_reward, rewards[0]}, is_done);
    }

    std::shared_ptr<Space> OccupancyMDP::getActionSpaceAt(const std::shared_ptr<State> &ostate, number t)
    {
        // If the action space corresponding to this ostate and t does not exist:
        if (ostate->toOccupancyState()->getActionSpaceAt(t) == nullptr)
        {
            // Compute the action space at this occupancy state and timestep
            std::shared_ptr<Space> joint_ddr_space = this->computeActionSpaceAt(ostate, t);

            // If we don't store action spaces
            if (!this->store_actions_)
            {
                return joint_ddr_space;
            }
            else
            {
                // Store the action space for state o
                ostate->toOccupancyState()->setActionSpaceAt(t, joint_ddr_space);
            }
        }
        // Return the action space corresponding to this ostate and t.
        return ostate->toOccupancyState()->getActionSpaceAt(t);
    }

    std::shared_ptr<Space> OccupancyMDP::getActionSpaceAt(const std::shared_ptr<Observation> &ostate, number t)
    {
        return this->getActionSpaceAt(ostate->toState(), t);
    }

    std::shared_ptr<Space> OccupancyMDP::computeActionSpaceAt(const std::shared_ptr<State> &ostate, number t)
    {
        switch (ostate->getTypeState())
        {
        case TypeState::OCCUPANCY_STATE :
            return this->computeActionSpaceAtOccupancy(ostate,t);
            break;
        case TypeState::SERIAL_OCCUPANCY_STATE :
            return this->computeActionSpaceAtSerial(ostate,t);
            break;
        default:
            throw sdm::exception::Exception("OccupancyMDP::computeActionSpaceAt error, the state doesn't use a formalism acceptable!");
            break;
        }
    }
    
    std::shared_ptr<Space> OccupancyMDP::computeActionSpaceAtOccupancy(const std::shared_ptr<State> &ostate, number t)
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
            std::shared_ptr<Space> individual_action_space = std::static_pointer_cast<MultiDiscreteSpace>(this->getUnderlyingProblem()->getActionSpace(t))->get(agent);
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

    std::shared_ptr<Space> OccupancyMDP::computeActionSpaceAtSerial(const std::shared_ptr<State> &ostate, number t)
    {
        // number ag_id = ostate->toSerialOccupancyState()->getCurrentAgentId();

        // // Get individual histories of agent i.
        // std::set<std::shared_ptr<HistoryInterface>> individual_histories = ostate->toOccupancyState()->getIndividualHistories(agent);
        // // Get individual history space of agent i.
        // std::shared_ptr<Space> individual_history_space = std::make_shared<DiscreteSpace>(sdm::tools::set2vector(individual_histories));
        // // Get action space of agent i.
        // std::shared_ptr<Space> individual_action_space = std::static_pointer_cast<MultiDiscreteSpace>(this->getUnderlyingProblem()->getActionSpace(t))->get(agent);
        // // Get individual ddr of agent i.
        // std::shared_ptr<Space> individual_ddr_space = std::make_shared<FunctionSpace<DeterministicDecisionRule>>(individual_history_space, individual_action_space, this->store_actions_);
        // // Add it to the corresponding vector.
        // individual_ddr_spaces.push_back(individual_ddr_space);

    }


    Pair<std::shared_ptr<State>, double> OccupancyMDP::computeNextStateAndProbability(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        return {this->computeNextState(ostate, action, observation, t), 1};
    }

    std::shared_ptr<State> OccupancyMDP::computeNextState(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        // The new fully uncompressed occupancy state
        std::shared_ptr<State> fully_uncompressed_next_occupancy_state = std::make_shared<OccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());
        // The new one step left occupancy state
        std::shared_ptr<State> one_step_left_compressed_next_occupancy_state = std::make_shared<OccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());
        //
        if (this->batch_size_ == 0)
        {
            std::tie(fully_uncompressed_next_occupancy_state, one_step_left_compressed_next_occupancy_state) = this->computeExactNextState(ostate, action, observation, t);
        }
        else
        {
            std::tie(fully_uncompressed_next_occupancy_state, one_step_left_compressed_next_occupancy_state) = this->computeSampledNextState(ostate, action, observation, t);
        }

        if (this->compression_)
        {
            // The new compressed occupancy state
            std::shared_ptr<State> compressed_next_occupancy_state;
            // Compress the occupancy state
            compressed_next_occupancy_state = one_step_left_compressed_next_occupancy_state->toOccupancyState()->compress();
            // std::cout << "15 - compressed_next_occupancy_state=" << compressed_next_occupancy_state->str() << std::endl;
            compressed_next_occupancy_state->toOccupancyState()->setFullyUncompressedOccupancy(fully_uncompressed_next_occupancy_state->toOccupancyState());
            compressed_next_occupancy_state->toOccupancyState()->setOneStepUncompressedOccupancy(one_step_left_compressed_next_occupancy_state->toOccupancyState());
            // std::cout << "17" << std::endl;
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

        // std::cout << "Compute Next Exact State" << std::endl;
        // std::cout << ostate->str() << std::endl;
        // The new fully uncompressed occupancy state
        std::shared_ptr<OccupancyStateInterface> fully_uncompressed_next_occupancy_state = std::make_shared<OccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());
        // The new one step left occupancy state
        std::shared_ptr<OccupancyStateInterface> one_step_left_compressed_next_occupancy_state = std::make_shared<OccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());

        // For each joint history in the support of the fully uncompressed occupancy state
        for (const auto &joint_history : ostate->toOccupancyState()->getFullyUncompressedOccupancy()->getJointHistories())
        {
            // std::cout << "1" << std::endl;
            // Apply the joint decision rule at joint_history to get the joint_action
            auto joint_action = this->applyDecisionRule(ostate->toOccupancyState()->toOccupancyState(), joint_history, action->toDecisionRule(), t);

            // std::cout << "2" << std::endl;
            // For each accessible belief at joint_history
            auto belief = ostate->toOccupancyState()->getFullyUncompressedOccupancy()->getBeliefAt(joint_history);

            // std::cout << "3" << std::endl;
            // For each observation in
            for (auto &joint_observation : *this->getUnderlyingMPOMDP()->getObservationSpace(t))
            {
                // std::cout << "4" << std::endl;
                // Get the next joint history
                auto next_joint_history = joint_history->expand(joint_observation->toObservation());

                // std::cout << "5" << std::endl;
                // Get the next belief
                auto next_belief = this->getUnderlyingBeliefMDP()->nextBelief(belief, joint_action, joint_observation->toObservation(), t);

                // std::cout << "6" << std::endl;
                // Compute the probability of next history, i.e. p(o') = p(o) * p(z | b, a)
                double next_joint_history_probability = ostate->toOccupancyState()->getFullyUncompressedOccupancy()->getProbability(joint_history) * this->getUnderlyingBeliefMDP()->getObservationProbability(belief, joint_action, next_belief->toBelief(), joint_observation->toObservation(), t);

                // If the next history probability is not zero
                if (next_joint_history_probability > 0)
                {
                    // std::cout << "7" << std::endl;
                    // Build fully uncompressed occupancy state
                    fully_uncompressed_next_occupancy_state->toOccupancyState()->addProbability(next_joint_history->toJointHistory(), next_belief->toBelief(), next_joint_history_probability);

                    // std::cout << "8" << std::endl;
                    // Update the probability of being in this next history (for the one step left uncompressed occupancy state)
                    auto compressed_joint_history = ostate->toOccupancyState()->getCompressedJointHistory(joint_history);

                    // std::cout << "9" << std::endl;
                    auto next_compressed_joint_history = compressed_joint_history->expand(joint_observation->toObservation());
                    one_step_left_compressed_next_occupancy_state->toOccupancyState()->addProbability(next_compressed_joint_history->toJointHistory(), next_belief->toBelief(), next_joint_history_probability);

                    // std::cout << "10" << std::endl;
                    // Update next history labels
                    one_step_left_compressed_next_occupancy_state->toOccupancyState()->updateJointLabels(next_joint_history->toJointHistory()->getIndividualHistories(), next_compressed_joint_history->toJointHistory()->getIndividualHistories());
                    // std::cout << "11" << std::endl;
                }
            }
        }

        // std::cout << "12" << std::endl;
        fully_uncompressed_next_occupancy_state->toOccupancyState()->finalize();
        // std::cout << "13" << std::endl;
        one_step_left_compressed_next_occupancy_state->toOccupancyState()->finalize();
        // std::cout << "14" << std::endl;

        // std::cout << "14 - fully_uncompressed_next_occupancy_state=" << fully_uncompressed_next_occupancy_state->str() << std::endl;
        // std::cout << "14 - one_step_left_compressed_next_occupancy_state=" << one_step_left_compressed_next_occupancy_state->str() << std::endl;

        return std::make_pair(fully_uncompressed_next_occupancy_state, one_step_left_compressed_next_occupancy_state);
    }

    Pair<std::shared_ptr<State>, std::shared_ptr<State>> OccupancyMDP::computeSampledNextState(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &, number t)
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
        return BaseBeliefMDP<OccupancyState>::nextBelief(belief, action, observation, t);
    }

    std::shared_ptr<Action> OccupancyMDP::applyDecisionRule(const std::shared_ptr<OccupancyStateInterface> &ostate, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<Action> &decision_rule, number t) const
    {
        // Get the list of individual history labels
        auto joint_labels = ostate->toOccupancyState()->getJointLabels(joint_history->getIndividualHistories()).toJoint<State>();
        // auto joint_labels = ostate->toOccupancyState()->getCompressedJointHistory(joint_history->getIndividualHistories());
        // auto joint_action = decision_rule->act(joint_labels);

        // Get the selected joint action
        auto action = std::static_pointer_cast<JointDeterministicDecisionRule>(decision_rule)->act(joint_labels);

        // Transform the selected joint action into joint action address
        auto joint_action = std::static_pointer_cast<Joint<std::shared_ptr<Action>>>(action);
        // Get the adress of the joint action object from the space of available joint action object.
        auto joint_action_address = std::static_pointer_cast<MultiDiscreteSpace>(this->getUnderlyingProblem()->getActionSpace(t))->getItemAddress(*joint_action->toJoint<Item>());
        return joint_action_address->toAction();
    }

    std::shared_ptr<HistoryInterface> OccupancyMDP::getNextHistory(const std::shared_ptr<Observation> &observation)
    {
        if (this->batch_size_ == 0)
        {
            return this->current_history_->expand(observation);
        }
        else
        {
            auto [jh, b] = this->current_state_->toOccupancyState()->sampleJointHistoryBelief();
            this->getUnderlyingProblem()->setInternalState(b->sampleState());
            return jh;
        }
    }

    double OccupancyMDP::getReward(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t)
    {
        double reward = 0;
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
        return reward;
    }

    double OccupancyMDP::getRewardBelief(const std::shared_ptr<BeliefInterface> &state, const std::shared_ptr<Action> &action, number t)
    {
        return this->getUnderlyingBeliefMDP()->getReward(state, action, t);
    }

    double OccupancyMDP::getExpectedNextValue(const std::shared_ptr<ValueFunction> &value_function, const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &joint_decision_rule, number t)
    {
        return value_function->getValueAt(this->nextOccupancyState(occupancy_state, joint_decision_rule, nullptr, t), t + 1);
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

    // void OccupancyMDP::setInitialState(const std::shared_ptr<State> &state)
    // {
    //     // Initialize empty state
    //     auto initial_state = std::make_shared<OccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());
    //     initial_state->toOccupancyState()->setProbability(this->initial_history_->toJointHistory(), state->toBelief(), 1);

    //     initial_state->toOccupancyState()->finalize();
    //     initial_state->toOccupancyState()->setFullyUncompressedOccupancy(initial_state);
    //     initial_state->toOccupancyState()->setOneStepUncompressedOccupancy(initial_state);

    //     this->initial_state_ = std::static_pointer_cast<State>(std::make_shared<OccupancyStateGraph>(initial_state));
    //     std::dynamic_pointer_cast<OccupancyStateGraph>(this->initial_state_)->initialize();
    // }
} // namespace sdm
