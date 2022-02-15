#include <memory>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/world/registry.hpp>

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
    BaseOccupancyMDP<TOccupancyState>::BaseOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &decpomdp, int memory, bool store_states, bool store_actions, int batch_size)
        : decpomdp(decpomdp), memory(memory)
    {
        this->store_states_ = store_states;
        this->store_actions_ = store_actions;
        this->batch_size_ = batch_size;
        this->mdp = decpomdp;

        // Initialize underlying belief mdp
        this->belief_mdp_ = std::make_shared<BeliefMDP>(decpomdp, batch_size, false, false);

        // Initialize initial history
        this->initial_history_ = std::make_shared<JointHistoryTree>(this->mdp->getNumAgents(), this->memory);

        // Initialize initial occupancy state
        this->initial_state_ = std::make_shared<TOccupancyState>(this->mdp->getNumAgents(), 0);

        this->initial_state_->toOccupancyState()->setProbability(this->initial_history_->toJointHistory(), this->belief_mdp_->getInitialState()->toBelief(), 1);
        this->initial_state_->toOccupancyState()->finalize();

        // Initialize Transition Graph
        this->mdp_graph_ = std::make_shared<Graph<std::shared_ptr<State>, Pair<std::shared_ptr<Action>, std::shared_ptr<Observation>>>>();
        this->mdp_graph_->addNode(this->initial_state_);

        this->reward_graph_ = std::make_shared<Graph<double, Pair<std::shared_ptr<State>, std::shared_ptr<Action>>>>();
        this->reward_graph_->addNode(0.0);
    }

    template <class TOccupancyState>
    BaseOccupancyMDP<TOccupancyState>::BaseOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &decpomdp, Config config)
        : BaseOccupancyMDP<TOccupancyState>(decpomdp,
                                            config.get("memory", 0),
                                            config.get("store_states", true),
                                            config.get("store_actions", true),
                                            config.get("batch_size", 0))
    {
        auto opt_int = config.getOpt<int>("state_type");
        auto opt_str = config.getOpt<std::string>("state_type");
        if (opt_int.has_value())
            this->setStateType((StateType)opt_int.value());
        else if (opt_str.has_value())
        {
            auto iter = STATE_TYPE_MAP.find(opt_str.value());
            this->setStateType((iter != STATE_TYPE_MAP.end()) ? iter->second : StateType::COMPRESSED);
        }
    }

    template <class TOccupancyState>
    BaseOccupancyMDP<TOccupancyState>::BaseOccupancyMDP(Config config)
        : BaseOccupancyMDP<TOccupancyState>(std::dynamic_pointer_cast<MPOMDPInterface>(sdm::world::createFromConfig(config)), config)
    {
    }

    template <class TOccupancyState>
    std::shared_ptr<Space> BaseOccupancyMDP<TOccupancyState>::getObservationSpaceAt(const std::shared_ptr<State> &, const std::shared_ptr<Action> &, number)
    {
        return std::make_shared<DiscreteSpace>(std::vector({sdm::NO_OBSERVATION}));
    }

    template <class TOccupancyState>
    void BaseOccupancyMDP<TOccupancyState>::setStateType(const StateType &state_type)
    {
        this->state_type = state_type;
        this->initial_state_->toOccupancyState()->setStateType(state_type);
    }

    template <class TOccupancyState>
    bool BaseOccupancyMDP<TOccupancyState>::checkCompatibility(const std::shared_ptr<Observation> &, const std::shared_ptr<Observation> &)
    {
        return true;
    }

    template <class TOccupancyState>
    std::shared_ptr<MPOMDPInterface> BaseOccupancyMDP<TOccupancyState>::getUnderlyingMPOMDP() const
    {
        return this->decpomdp;
    }

    template <class TOccupancyState>
    std::shared_ptr<BeliefMDPInterface> BaseOccupancyMDP<TOccupancyState>::getUnderlyingBeliefMDP()
    {
        return this->belief_mdp_;
    }

    template <class TOccupancyState>
    double BaseOccupancyMDP<TOccupancyState>::do_excess(double incumbent, double lb, double ub, double cost_so_far, double error, number horizon)
    {
        // return std::min(ub - lb, cost_so_far + this->mdp->getDiscount(horizon) * ub - incumbent) - error / this->getWeightedDiscount(horizon);
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

        bool is_done = (this->getHorizon() > 0) ? (this->step_ >= this->getHorizon()) : false;
        return std::make_tuple(this->current_state_, std::vector<double>(this->mdp->getNumAgents(), occupancy_reward), is_done);
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
        std::vector<std::shared_ptr<DecisionRule>> a;
        for (int agent = 0; agent < this->mdp->getNumAgents(); agent++)
        {
            // Input states for the a of agent.
            std::vector<std::shared_ptr<Item>> inputs;
            // Outputed actions for each of these.
            std::vector<std::shared_ptr<Item>> outputs;
            for (const auto &individual_history : ostate->getIndividualHistories(agent))
            {
                inputs.push_back(individual_history);
                outputs.push_back(this->decpomdp->getActionSpace(agent, t)->sample());
            }
            a.push_back(std::make_shared<DeterministicDecisionRule>(inputs, outputs));
        }
        return std::make_shared<JointDeterministicDecisionRule>(a, this->mdp->getActionSpace(t));
    }

    template <class TOccupancyState>
    std::shared_ptr<Space> BaseOccupancyMDP<TOccupancyState>::computeActionSpaceAt(const std::shared_ptr<State> &ostate, number t)
    {
        // return ostate->getActionSpace(ostate);
        
        // Vector of individual deterministic decision rules of each agent.
        std::vector<std::shared_ptr<Space>> individual_ddr_spaces;
        // For each agent from 0 to N-1:
        for (int agent = 0; agent < this->mdp->getNumAgents(); agent++)
        {
            // Get individual histories of agent i.
            std::set<std::shared_ptr<HistoryInterface>> individual_histories = ostate->toOccupancyState()->getIndividualHistories(agent);
            // Get individual history space of agent i.
            std::shared_ptr<Space> individual_history_space = std::make_shared<DiscreteSpace>(sdm::tools::set2vector(individual_histories));
            // Get action space of agent i.
            std::shared_ptr<Space> individual_action_space = this->decpomdp->getActionSpace(agent, t);
            // Get individual ddr of agent i.
            std::shared_ptr<Space> individual_ddr_space = std::make_shared<FunctionSpace<DeterministicDecisionRule>>(individual_history_space, individual_action_space, this->store_actions_);
            // Add it to the corresponding vector.
            individual_ddr_spaces.push_back(individual_ddr_space);
        }

        // Create the function space of joint deterministic decision rules.
        std::shared_ptr<Space> joint_ddr_space = std::make_shared<FunctionSpace<JointDeterministicDecisionRule>>(
            std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{nullptr}),
            std::make_shared<MultiDiscreteSpace>(individual_ddr_spaces, this->store_actions_),
            this->store_actions_,
            this->decpomdp->getActionSpace(t));

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
            // return this->computeExactNextState(ostate, action, observation, t);
            auto next_state = ostate->next(this->mdp, action, observation, t);
            return next_state;
        }
        else
        {
            // Compute sampled next state
            return this->computeSampledNextState(ostate, action, observation, t);
        }

    }

    // template <class TOccupancyState>
    // Pair<std::shared_ptr<State>, double> BaseOccupancyMDP<TOccupancyState>::computeSampledNextState(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    // {
    //     // The new fully uncompressed occupancy state
    //     std::shared_ptr<OccupancyStateInterface> next_fully_uncompressed_occupancy_state = std::make_shared<TOccupancyState>(this->mdp->getNumAgents());

    //     // The new one step left occupancy state
    //     std::shared_ptr<OccupancyStateInterface> next_one_step_left_compressed_occupancy_state = std::make_shared<TOccupancyState>(this->mdp->getNumAgents());

    //     // Bag for next history (w) and next state (y) and counters for the two.
    //     std::unordered_map<std::shared_ptr<HistoryInterface>, Pair<double, std::unordered_map<std::shared_ptr<State>, double>>> w_y_bag;

    //     // This map is for keeping track of the joint history and next observation for each next joint history.
    //     std::unordered_map<std::shared_ptr<HistoryInterface>, Tuple<std::shared_ptr<HistoryInterface>, std::shared_ptr<BeliefInterface>, std::shared_ptr<Action>, std::shared_ptr<Observation>>> w__o_b_u_z__map;

    //     //
    //     std::shared_ptr<State> true_state = this->mdp->getInternalState();
    //     //
    //     for (int k = 0; k < this->batch_size_; k++)
    //     {
    //         // Sample joint history and belief.
    //         auto [joint_history, belief] = ostate->toOccupancyState()->sampleJointHistoryBelief();
    //         // Sample state.
    //         auto state = belief->sampleState();
    //         // Get joint action.
    //         auto joint_action = action->toDecisionRule()->act(joint_history);
    //         // Set state.
    //         this->mdp->setInternalState(state);
    //         // Sample next observation.
    //         auto [next_joint_observation, _, __] = this->mdp->step(joint_action, false);
    //         if (this->checkCompatibility(next_joint_observation, observation))
    //         {
    //             // Sample next state.
    //             auto next_state = this->mdp->getInternalState();
    //             // Sample next joint history.
    //             auto next_joint_history = joint_history->expand(next_joint_observation);
    //             // If next_joint_history is seen for the first time:
    //             if (w_y_bag.find(next_joint_history) == w_y_bag.end())
    //             {
    //                 // Set next_joint_history's counter to 1.
    //                 w_y_bag[next_joint_history].first = 1;
    //                 // No need to check if (next_joint_history, next_state) was seen before, the answer is no.
    //                 // Set (next_joint_history, next_state)'s counter to 1.
    //                 w_y_bag[next_joint_history].second[next_state] = 1;
    //                 // These two will be needed later.
    //                 w__o_b_u_z__map[next_joint_history] = std::make_tuple(joint_history, belief, joint_action, next_joint_observation);
    //             }
    //             // If next_joint_history was seen before:
    //             else
    //             {
    //                 // Incriment next_joint_history's counter.
    //                 w_y_bag[next_joint_history].first++;
    //                 // If (next_joint_history, next_state) is seen for the first time:
    //                 if (w_y_bag[next_joint_history].second.find(next_state) == w_y_bag[next_joint_history].second.end())
    //                 {
    //                     // Set (next_joint_history, next_state)'s counter to 1.
    //                     w_y_bag[next_joint_history].second[next_state] = 1;
    //                 }
    //                 // If (next_joint_history, next_state) was seen before:
    //                 else
    //                 {
    //                     // Incriment (next_joint_history, next_state)'s counter.
    //                     w_y_bag[next_joint_history].second[next_state]++;
    //                 }
    //             }
    //         }
    //     }

    //     // Iterate through the (next_joint_history, next_state) pairs.
    //     for (auto const [next_joint_history, next_joint_history_count__y_bag] : w_y_bag)
    //     {
    //         auto [next_joint_history_count, y_bag] = next_joint_history_count__y_bag;
    //         auto [joint_history, belief, joint_action, observation] = w__o_b_u_z__map[next_joint_history];
    //         std::shared_ptr<BeliefInterface> next_belief;
    //         auto belief_graph = this->belief_mdp_->getMDPGraph();
    //         auto successor = belief_graph->getNode(belief)->getSuccessor(std::make_pair(joint_action, observation));
    //         // If already in the successor list
    //         if (successor != nullptr)
    //         {
    //             // Return the successor node
    //             next_belief = successor->getData()->toBelief();
    //         }
    //         else
    //         {
    //             next_belief = std::make_shared<Belief>();

    //             for (auto const [next_state, next_state_count] : y_bag)
    //             {
    //                 next_belief->setProbability(next_state, next_state_count / next_joint_history_count);
    //             }
    //         }
    //         next_belief->finalize();
    //         belief_graph->addNode(next_belief);
    //         next_fully_uncompressed_occupancy_state->addProbability(next_joint_history->toJointHistory(), next_belief, next_joint_history_count / this->batch_size_);
    //         auto joint_history_ = next_joint_history->getPreviousHistory();
    //         auto observation_ = next_joint_history->getLastObservation();
    //         auto compressed_joint_history = ostate->toOccupancyState()->getCompressedJointHistory(joint_history->toJointHistory());
    //         auto next_compressed_joint_history = compressed_joint_history->expand(observation->toObservation());
    //         next_one_step_left_compressed_occupancy_state->addProbability(next_compressed_joint_history->toJointHistory(), next_belief->toBelief(), next_joint_history_count / this->batch_size_);
    //         next_one_step_left_compressed_occupancy_state->updateJointLabels(next_joint_history->toJointHistory()->getIndividualHistories(), next_compressed_joint_history->toJointHistory()->getIndividualHistories());
    //     }
    //     //
    //     this->mdp->setInternalState(true_state);

    //     return this->finalizeNextState(next_one_step_left_compressed_occupancy_state, next_fully_uncompressed_occupancy_state, t);
    // }

    template <class TOccupancyState>
    std::shared_ptr<BaseOccupancyMDP<TOccupancyState>> BaseOccupancyMDP<TOccupancyState>::getptr()
    {
        return std::enable_shared_from_this<BaseOccupancyMDP<TOccupancyState>>::shared_from_this();
    }

} // namespace sdm
