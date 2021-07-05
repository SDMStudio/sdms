#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/core/state/jhistory_tree.hpp>
#include <sdm/core/space/function_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/action/det_decision_rule.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>

#include <sdm/core/state/occupancy_state_graph.hpp>


namespace sdm
{

    OccupancyMDP::OccupancyMDP() 
    {

    }

    OccupancyMDP::OccupancyMDP(std::shared_ptr<MPOMDPInterface> underlying_dpomdp, number max_history_length)
        : BeliefMDP(underlying_dpomdp)
    {
        // Initialize history
        this->initial_history_ = std::make_shared<JointHistoryTree>(this->getUnderlyingMPOMDP()->getNumAgents(), (max_history_length > 0) ? max_history_length : -1);

        // Initialize empty state
        this->setInitialState(this->initial_state_);
    }

    std::shared_ptr<Observation> OccupancyMDP::reset()
    {
        this->current_history_ = this->initial_history_;
        return BeliefMDP::reset();
    }

    std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> OccupancyMDP::step(std::shared_ptr<Action> action)
    {
        auto joint_action = this->applyDecisionRule(this->current_state_->toOccupancyState(), this->current_history_->toJointHistory(), action, this->step_);
        auto feedback = std::dynamic_pointer_cast<MDP>(this->getUnderlyingProblem())->step(joint_action);
        auto next_obs = std::get<0>(feedback);
        this->current_state_ = this->nextState(this->current_state_, action, this->step_);
        this->current_history_ = this->current_history_->expand(next_obs);
        this->step_++;
        return std::make_tuple(this->current_state_, std::get<1>(feedback), std::get<2>(feedback));
    }

    std::shared_ptr<Space> OccupancyMDP::getActionSpaceAt(const std::shared_ptr<State> &ostate, number t)
    {
        std::vector<std::shared_ptr<Space>> individual_ddr_spaces;
        for (int agent_id = 0; agent_id < this->getUnderlyingProblem()->getNumAgents(); agent_id++)
        {
            // Get history space of agent i
            auto individual_history = ostate->toOccupancyState()->getIndividualHistories(agent_id);
            auto individual_history_space = std::make_shared<DiscreteSpace>(sdm::tools::set2vector(individual_history));
            // Get action space of agent i
            auto individual_action_space = std::static_pointer_cast<MultiDiscreteSpace>(this->getUnderlyingProblem()->getActionSpace(t))->get(agent_id);
            // Add individual decision rule space of agent i
            individual_ddr_spaces.push_back(std::make_shared<FunctionSpace<DeterministicDecisionRule>>(individual_history_space, individual_action_space, false));
        }

        // Now we can return a discrete space of all joint decision rules
        std::shared_ptr<Space> null_space = std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{std::make_shared<DiscreteState>(3)});
        std::shared_ptr<Space> joint_ddr_space = std::make_shared<MultiDiscreteSpace>(individual_ddr_spaces, false);

        auto function_space = std::make_shared<FunctionSpace<JointDeterministicDecisionRule>>(null_space, joint_ddr_space, false);

        return function_space;
    }

    std::shared_ptr<Space> OccupancyMDP::getActionSpaceAt(const std::shared_ptr<Observation> &ostate, number t)
    {
        std::vector<std::shared_ptr<Space>> individual_ddr_spaces;
        for (int agent_id = 0; agent_id < this->getUnderlyingProblem()->getNumAgents(); agent_id++)
        {
            // Get history space of agent i
            std::set<std::shared_ptr<HistoryInterface>> individual_histories = ostate->toState()->toOccupancyState()->getIndividualHistories(agent_id);
            // Get individual history space of agent i
            std::shared_ptr<Space> individual_history_space = std::make_shared<DiscreteSpace>(sdm::tools::set2vector(individual_histories));
            // Get action space of agent i
            std::shared_ptr<Space> individual_action_space = std::static_pointer_cast<MultiDiscreteSpace>(this->getUnderlyingProblem()->getActionSpace(t))->get(agent_id);
            // Get individual ddr of agent i
            std::shared_ptr<Space> individual_ddr_space = std::make_shared<FunctionSpace<DeterministicDecisionRule>>(individual_history_space, individual_action_space, true);
            // Add it to the corresponding vector
            individual_ddr_spaces.push_back(individual_ddr_space);
        }

        // Now we can return a discrete space of all joint decision rules
        std::shared_ptr<Space> joint_ddr_space = std::make_shared<FunctionSpace<JointDeterministicDecisionRule>>(
            std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{std::make_shared<DiscreteState>(3)}), 
            std::make_shared<MultiDiscreteSpace>(individual_ddr_spaces, true), 
            true
        );
        return joint_ddr_space;
    }

    std::shared_ptr<State> OccupancyMDP::nextOccupancy(const std::shared_ptr<State> &state_tmp, const std::shared_ptr<Action> &action_tmp, const std::shared_ptr<Observation>&, number t, bool compression) const
    {
        auto belief_and_eta = this->nextElement(state_tmp->toBelief(),action_tmp,nullptr,t,compression);
        return std::static_pointer_cast<OccupancyStateGraph>(state_tmp->toOccupancyState())->next(belief_and_eta.first,belief_and_eta.second, this->getUnderlyingMPOMDP(), action_tmp, nullptr, t);
    }

    std::shared_ptr<State> OccupancyMDP::nextState(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &joint_idr, number h, const std::shared_ptr<HSVI> &) const
    {
        return this->nextOccupancy(ostate, joint_idr,nullptr, h, true);
    }

    Pair<std::shared_ptr<BeliefInterface>, double> OccupancyMDP::nextElement(const std::shared_ptr<BeliefInterface> &belief_tmp, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &, number t,bool compression) const
    {
        try
        {
            auto under_pb = std::dynamic_pointer_cast<POMDPInterface>(this->getUnderlyingProblem());

            auto occupancy_state = belief_tmp->toOccupancyState();
            auto decision_rule = action->toDecisionRule();

            // The new compressed occupancy state
            std::shared_ptr<OccupancyStateInterface> new_compressed_occupancy_state;
            // The new fully uncompressed occupancy state
            std::shared_ptr<OccupancyStateInterface> new_fully_uncompressed_occupancy_state = std::make_shared<OccupancyState>(under_pb->getNumAgents());
            // The new one step left occupancy state
            std::shared_ptr<OccupancyStateInterface> new_one_step_left_compressed_occupancy_state = std::make_shared<OccupancyState>(under_pb->getNumAgents());

            for (const auto &joint_history : occupancy_state->getFullyUncompressedOccupancy()->getJointHistories())
            {
                auto joint_action = this->applyDecisionRule(occupancy_state->toOccupancyState(), joint_history, decision_rule, t);
                
                for (const auto &belief : occupancy_state->getFullyUncompressedOccupancy()->getBeliefsAt(joint_history))
                {
                    for (auto &joint_observation : *under_pb->getObservationSpace(t))
                    {
                        auto next_joint_history = joint_history->expand(joint_observation->toObservation());
                        auto next_belief = BeliefMDP::nextBelief(belief, joint_action, joint_observation->toObservation(), t);

                        // p(o') = p(o) * p(z | b, a)
                        double proba_next_history = occupancy_state->getFullyUncompressedOccupancy()->getProbability(joint_history, belief) * BeliefMDP::getObservationProbability(belief, joint_action, nullptr, joint_observation->toObservation(), t);

                        if (proba_next_history > 0)
                        {
                            // Build fully uncompressed occupancy state
                            new_fully_uncompressed_occupancy_state->addProbability(next_joint_history->toJointHistory(), next_belief->toBelief(), proba_next_history);

                            // Build one step left uncompressed occupancy state
                            auto compressed_joint_history = occupancy_state->getCompressedJointHistory(joint_history);

                            auto next_compressed_joint_history = compressed_joint_history->expand(joint_observation->toObservation());
                            new_one_step_left_compressed_occupancy_state->addProbability(next_compressed_joint_history->toJointHistory(), next_belief->toBelief(), proba_next_history);

                            // Update next history labels
                            new_one_step_left_compressed_occupancy_state->updateJointLabels(next_joint_history->toJointHistory()->getIndividualHistories(), next_compressed_joint_history->toJointHistory()->getIndividualHistories());
                        }
                    }
                }
            }

            // Finalize the one step left compressed occupancy state
            new_one_step_left_compressed_occupancy_state->finalize();
            new_fully_uncompressed_occupancy_state->finalize();

            if (compression)
            {
                // Compress the occupancy state
                new_compressed_occupancy_state = new_one_step_left_compressed_occupancy_state->compress();
                new_compressed_occupancy_state->setFullyUncompressedOccupancy(new_fully_uncompressed_occupancy_state);
                new_compressed_occupancy_state->setOneStepUncompressedOccupancy(new_one_step_left_compressed_occupancy_state);

                return std::make_pair(new_compressed_occupancy_state,1);
            }

            new_one_step_left_compressed_occupancy_state->setFullyUncompressedOccupancy(new_fully_uncompressed_occupancy_state);
            new_one_step_left_compressed_occupancy_state->setOneStepUncompressedOccupancy(new_one_step_left_compressed_occupancy_state);
            return std::make_pair(new_one_step_left_compressed_occupancy_state,1);
        }
        catch (const std::exception &exc)
        {
            // catch anything thrown within try block that derives from std::exception
            std::cerr << "OccupancyMDP::nextState(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
    }

    std::shared_ptr<Action> OccupancyMDP::applyDecisionRule(const std::shared_ptr<OccupancyStateInterface> &ostate, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<Action> &decision_rule, number t) const
    {
        // Get list of individual history labels
        auto joint_labels = ostate->toOccupancyState()->getJointLabels(joint_history->getIndividualHistories()).toJoint<State>();

        // Get selected joint action
        auto action = std::static_pointer_cast<JointDeterministicDecisionRule>(decision_rule)->act(joint_labels);

        // Transform selected joint action into joint action address
        auto joint_action = std::static_pointer_cast<Joint<std::shared_ptr<Action>>>(action);
        auto joint_action_address = std::static_pointer_cast<MultiDiscreteSpace>(this->getUnderlyingProblem()->getActionSpace(t))->getItemAddress(*joint_action->toJoint<Item>());
        return joint_action_address->toAction();
    }

    double OccupancyMDP::getReward(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t) const
    {
        double reward = 0;
        for (const auto &joint_history : occupancy_state->toOccupancyState()->getJointHistories())
        {
            for (const auto &belief : occupancy_state->toOccupancyState()->getBeliefsAt(joint_history))
            {
                auto joint_action = this->applyDecisionRule(occupancy_state->toOccupancyState(), joint_history, decision_rule, t);
                reward += occupancy_state->toOccupancyState()->getProbability(joint_history, belief) * BeliefMDP::getReward(belief, joint_action, t);
            }
        }

        return reward;
    }

    double OccupancyMDP::getRewardBelief(const std::shared_ptr<BeliefInterface> &state, const std::shared_ptr<Action> &action, number t) const
    {
        return BeliefMDP::getReward(state,action,t);
    }

    double OccupancyMDP::getExpectedNextValue(const std::shared_ptr<ValueFunction> &value_function, const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &joint_decision_rule, number t) const
    {
        // std::cout<<"Action "<<joint_decision_rule->str()<<std::endl;
        // std::cout<<"Next state "<<this->nextState(occupancy_state, joint_decision_rule, t)->str()<<std::endl;
        return value_function->getValueAt(this->nextState(occupancy_state, joint_decision_rule, t), t + 1);
    }

    std::shared_ptr<MPOMDPInterface> OccupancyMDP::getUnderlyingMPOMDP() const
    {
        return std::dynamic_pointer_cast<MPOMDPInterface>(this->getUnderlyingMDP());
    }

    void OccupancyMDP::setInitialState(const std::shared_ptr<State>& state)
    {
        // Initialize empty state
        auto initial_state = std::make_shared<OccupancyState>(this->getUnderlyingMPOMDP()->getNumAgents());
        initial_state->toOccupancyState()->setProbability(this->initial_history_->toJointHistory(), state->toBelief(), 1);

        initial_state->toOccupancyState()->finalize();
        initial_state->toOccupancyState()->setFullyUncompressedOccupancy(initial_state);
        initial_state->toOccupancyState()->setOneStepUncompressedOccupancy(initial_state);

        this->initial_state_ = std::static_pointer_cast<State>(std::make_shared<OccupancyStateGraph>(initial_state));
        std::dynamic_pointer_cast<OccupancyStateGraph>(this->initial_state_)->initialize();
    }
} // namespace sdm
