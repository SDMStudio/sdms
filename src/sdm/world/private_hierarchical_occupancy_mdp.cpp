#include <sdm/world/private_hierarchical_occupancy_mdp.hpp>

namespace sdm
{

    PrivateHierarchicalOccupancyMDP::PrivateHierarchicalOccupancyMDP()
    {

    }

    PrivateHierarchicalOccupancyMDP::PrivateHierarchicalOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &underlying_dpomdp, number memory, int batch_size, bool store_action_spaces)
        : OccupancyMDP(underlying_dpomdp, memory, batch_size, store_action_spaces)
    {
    }

    std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> PrivateHierarchicalOccupancyMDP::step(std::shared_ptr<Action> action)
    {
        auto joint_action = this->applyDecisionRule(this->current_state_->toOccupancyState(), this->current_history_->toJointHistory(), action, this->step_);
        auto [observation, rewards, is_done]  = this->getUnderlyingProblem()->step(joint_action);
        double occupancy_reward = this->getReward(this->current_state_, action, this->step_);
        std::shared_ptr<Observation> observation_n = std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(observation)->at(this->getUnderlyingMDP()->getNumAgents() - 1);
        this->current_state_ = this->nextOccupancyState(this->current_state_, action, observation_n, this->step_);
        this->current_history_ = this->getNextHistory(observation);
        this->step_++;
        return std::make_tuple(this->current_state_, std::vector<double>{occupancy_reward, rewards[0]}, is_done);
    }

    // std::shared_ptr<Space> PrivateHierarchicalOccupancyMDP::computeActionSpaceAt(const std::shared_ptr<State> &ostate, number t)
    // {
    //     // Get joint histories that are in the support of the occupancy state.
    //     std::set<std::shared_ptr<JointHistoryInterface>> joint_histories = ostate->toOccupancyState()->getJointHistories();
    //     // Vector for individual hierarchical histories of all agents from 1 to N.
    //     std::vector<std::set<std::shared_ptr<JointHistoryInterface>>> all_individual_hierarchical_histories;
    //     // For all agents from 1 to N:
    //     for (int agent = 0; agent < this->getUnderlyingProblem()->getNumAgents(); agent++)
    //     {
    //         // Individual hierarchical histories for Agent I.
    //         std::set<std::shared_ptr<JointHistoryInterface>> individual_hierarchical_histories;
    //         // For each possible joint history:
    //         for (std::shared_ptr<JointHistoryInterface> joint_history : joint_histories)
    //         {
    //             // 
    //             std::shared_ptr<JointHistoryInterface> individual_hierarchical_history = std::make_shared<JointHistoryTree>();
    //             // For each agent between agent I and agent N (both included):
    //             for (int lower_agent = agent; lower_agent < this->getUnderlyingProblem()->getNumAgents(); lower_agent++)
    //             {
    //                 individual_hierarchical_history->addIndividualHistory(std::dynamic_pointer_cast<std::shared_ptr<JointHistoryTree>>(joint_history)->get(lower_agent));
    //             }
    //             individual_hierarchical_histories.emplace(individual_hierarchical_history);
    //         }
    //         all_individual_hierarchical_histories.push_back(individual_hierarchical_histories);
    //     }


    //     // Vector of individual deterministic decision rules of each agent.
    //     std::vector<std::shared_ptr<Space>> individual_ddr_spaces;
    //     // For each agent from 0 to N-1:
    //     for (int agent = 0; agent < this->getUnderlyingProblem()->getNumAgents(); agent++)
    //     {
    //         // Get individual histories of agent i.
    //         std::set<std::shared_ptr<HistoryInterface>> individual_histories = ostate->toOccupancyState()->getIndividualHistories(agent);
    //         // Get individual history space of agent i.
    //         std::shared_ptr<Space> individual_history_space = std::make_shared<DiscreteSpace>(sdm::tools::set2vector(individual_histories));
    //         // Get action space of agent i.
    //         std::shared_ptr<Space> individual_action_space = std::static_pointer_cast<MultiDiscreteSpace>(this->getUnderlyingProblem()->getActionSpace(t))->get(agent);
    //         // Get individual ddr of agent i.
    //         std::shared_ptr<Space> individual_ddr_space = std::make_shared<FunctionSpace<DeterministicDecisionRule>>(individual_history_space, individual_action_space, this->store_action_spaces_);
    //         // Add it to the corresponding vector.
    //         individual_ddr_spaces.push_back(individual_ddr_space);
    //     }

    //     // Create the function space of joint deterministic decision rules.
    //     std::shared_ptr<Space> joint_ddr_space = std::make_shared<FunctionSpace<JointDeterministicDecisionRule>>(
    //         std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{std::make_shared<DiscreteState>(3)}),
    //         std::make_shared<MultiDiscreteSpace>(individual_ddr_spaces, this->store_action_spaces_),
    //         this->store_action_spaces_
    //     );
    //     return joint_ddr_space;
    // }

    Pair<std::shared_ptr<State>, std::shared_ptr<State>> PrivateHierarchicalOccupancyMDP::computeExactNextState(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> & observation_n, number t)
    {
        // The new fully uncompressed occupancy state
        std::shared_ptr<State> fully_uncompressed_next_occupancy_state = std::make_shared<OccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());
        // The new one step left occupancy state
        std::shared_ptr<State> one_step_left_compressed_next_occupancy_state = std::make_shared<OccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());

        for (const auto &joint_history : ostate->toOccupancyState()->getFullyUncompressedOccupancy()->getJointHistories())
        {
            auto joint_action = this->applyDecisionRule(ostate->toOccupancyState()->toOccupancyState(), joint_history, action, t);

            for (const auto &belief : ostate->toOccupancyState()->getFullyUncompressedOccupancy()->getBeliefsAt(joint_history))
            {
                for (auto &joint_observation : *this->getUnderlyingMPOMDP()->getObservationSpace(t))
                {
                    if (std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(joint_observation)->at(this->getUnderlyingMDP()->getNumAgents() - 1) == observation_n)
                    {
                        auto next_joint_history = joint_history->expand(joint_observation->toObservation());
                        auto next_belief = this->getUnderlyingBeliefMDP()->nextBelief(belief, joint_action, joint_observation->toObservation(), t);

                        // p(o') = p(o) * p(z | b, a)
                        double next_joint_history_probability = ostate->toOccupancyState()->getFullyUncompressedOccupancy()->getProbability(joint_history, belief) * this->getUnderlyingBeliefMDP()->getObservationProbability(belief, joint_action, next_belief->toBelief(), joint_observation->toObservation(), t);

                        if (next_joint_history_probability > 0)
                        {
                            // Build fully uncompressed occupancy state*
                            fully_uncompressed_next_occupancy_state->toOccupancyState()->addProbability(next_joint_history->toJointHistory(), next_belief->toBelief(), next_joint_history_probability);

                            // Build one step left uncompressed occupancy state
                            auto compressed_joint_history = ostate->toOccupancyState()->getCompressedJointHistory(joint_history);

                            auto next_compressed_joint_history = compressed_joint_history->expand(joint_observation->toObservation());
                            one_step_left_compressed_next_occupancy_state->toOccupancyState()->addProbability(next_compressed_joint_history->toJointHistory(), next_belief->toBelief(), next_joint_history_probability);

                            // Update next history labels
                            one_step_left_compressed_next_occupancy_state->toOccupancyState()->updateJointLabels(next_joint_history->toJointHistory()->getIndividualHistories(), next_compressed_joint_history->toJointHistory()->getIndividualHistories());
                        }
                    }
                }
            }
        }

        fully_uncompressed_next_occupancy_state->toBelief()->normalizeBelief(fully_uncompressed_next_occupancy_state->toBelief()->norm_1());
        one_step_left_compressed_next_occupancy_state->toBelief()->normalizeBelief(one_step_left_compressed_next_occupancy_state->toBelief()->norm_1());

        fully_uncompressed_next_occupancy_state->toOccupancyState()->finalize();
        one_step_left_compressed_next_occupancy_state->toOccupancyState()->finalize();
        
        return std::make_pair(fully_uncompressed_next_occupancy_state, one_step_left_compressed_next_occupancy_state);        
    }
    
    Pair<std::shared_ptr<State>, std::shared_ptr<State>> PrivateHierarchicalOccupancyMDP::computeSampledNextState(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> & observation_n, number t)
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
                next_belief =  successor->getData()->toBelief();
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

    
    
} // namespace sdm
