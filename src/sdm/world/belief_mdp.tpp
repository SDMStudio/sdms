#include <sdm/core/state/belief_state.hpp>
#include <sdm/utils/struct/graph.hpp>

namespace sdm
{

    template <class TBelief>
    BaseBeliefMDP<TBelief>::BaseBeliefMDP()
    {
    }

    // BaseBeliefMDP<TBelief>::BaseBeliefMDP(const std::shared_ptr<POMDPInterface> &pomdp, const Belief &initial_belief)
    //     : SolvableByMDP(pomdp)
    // {
    //     this->initial_state_ = std::make_shared<BeliefStateGraph>(initial_belief);
    //     this->current_state_ = std::make_shared<BeliefStateGraph>(*std::static_pointer_cast<TBelief>(this->initial_state_));
    // }

    template <class TBelief>
    BaseBeliefMDP<TBelief>::BaseBeliefMDP(const std::shared_ptr<POMDPInterface> &pomdp, int batch_size) : SolvableByMDP(pomdp)
    {
        this->batch_size_ = batch_size;
        auto initial_state = std::make_shared<TBelief>();

        // For each state at t=0:
        for (const auto &state : *pomdp->getStateSpace(0))
        {
            // Get the state's probability
            double probability = pomdp->getStartDistribution()->getProbability(state->toState(), nullptr);
            // If the state is possible:
            if (probability > 0)
            {
                // Set the probability
                initial_state->setProbability(state->toState(), probability);
            }
        }

        initial_state->finalize();
        this->initial_state_ = initial_state;
        this->mdp_graph_ = std::make_shared<Graph<std::shared_ptr<State>, Pair<std::shared_ptr<Action>, std::shared_ptr<Observation>>>>();
        this->mdp_graph_->addNode(this->initial_state_);

        this->reward_graph_ = std::make_shared<Graph<double, Pair<std::shared_ptr<State>, std::shared_ptr<Action>>>>();
        this->reward_graph_->addNode(0.0);
    }

    template <class TBelief>
    Pair<std::shared_ptr<State>, double> BaseBeliefMDP<TBelief>::computeNextStateAndProbability(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        // std::cout << "BaseBeliefMDP<TBelief>::computeNextStateAndProbability() " << std::endl;

        //
        std::shared_ptr<State> next_belief = this->computeNextState(belief, action, observation, t);
        // Compute the coefficient of normalization (eta)
        double eta = next_belief->toBelief()->norm_1();
        next_belief->toBelief()->normalizeBelief(eta);
        return {next_belief->toBelief(), eta};
    }

    template <class TBelief>
    std::shared_ptr<State> BaseBeliefMDP<TBelief>::computeNextState(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        // std::cout << "BaseBeliefMDP<TBelief>::computeNextState() " << std::endl;

        if (this->batch_size_ == 0)
        {
            return this->computeExactNextState(belief, action, observation, t).first;
        }
        else
        {
            return this->computeSampledNextState(belief, action, observation, t).first;
        }
    }

    template <class TBelief>
    Pair<std::shared_ptr<State>, std::shared_ptr<State>> BaseBeliefMDP<TBelief>::computeExactNextState(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        // std::cout << "BaseBeliefMDP<TBelief>::computeExactNextState() " << std::endl;

        // Create next belief.
        std::shared_ptr<State> next_belief = std::make_shared<TBelief>();
        // For each possible next state:
        for (const auto &next_state : *this->getUnderlyingPOMDP()->getStateSpace(t + 1))
        {
            // Set its probability to 0.
            double next_state_probability = 0;
            // For each possible state:
            for (const auto &state : *this->getUnderlyingPOMDP()->getStateSpace(t))
            {
                // Add to the the probability of transition * the probability of being in the state.
                next_state_probability += this->getUnderlyingPOMDP()->getDynamics(state->toState(), action, next_state->toState(), observation, t) * belief->toBelief()->getProbability(state->toState());
            }
            // If the next state is possible:
            if (next_state_probability > 0)
            {
                // Set its probability to the value found.
                next_belief->toBelief()->setProbability(next_state->toState(), next_state_probability);
            }
        }
        next_belief->toBelief()->finalize();
        // Return next belief.
        return std::make_pair(next_belief, nullptr);
    }

    template <class TBelief>
    Pair<std::shared_ptr<State>, std::shared_ptr<State>> BaseBeliefMDP<TBelief>::computeSampledNextState(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number)
    {
        // std::cout << "BaseBeliefMDP<TBelief>::computeSampledNextState() " << std::endl;

        // Create next belief.
        std::shared_ptr<State> next_belief = std::make_shared<TBelief>();
        //
        std::shared_ptr<State> true_state = this->getUnderlyingProblem()->getInternalState();
        //
        int k = 0;
        // while
        while (k < this->batch_size_)
        {
            std::shared_ptr<State> state = belief->toBelief()->sampleState();
            this->getUnderlyingProblem()->setInternalState(state);
            auto [possible_observation, rewards, is_done] = this->getUnderlyingProblem()->step(action, false);
            if (observation == possible_observation)
            {
                std::shared_ptr<State> next_state = this->getUnderlyingProblem()->getInternalState();
                next_belief->toBelief()->addProbability(next_state, 1.0 / double(this->batch_size_));
                k++;
            }
        }
        //
        this->getUnderlyingProblem()->setInternalState(true_state);

        // Finalize belief
        next_belief->toBelief()->finalize();

        // Return next belief.
        return std::make_pair(next_belief, nullptr);
    }

    template <class TBelief>
    std::shared_ptr<State> BaseBeliefMDP<TBelief>::nextBelief(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        // std::cout << "BaseBeliefMDP<TBelief>::nextBelief() " << std::endl;

        auto action_observation = std::make_pair(action, observation);

        // If we store data in the graph
        if ((this->store_states_) && (this->store_action_spaces_ || this->store_actions_))
        {
            // Get the successor
            auto successor = this->mdp_graph_->getNode(belief)->getSuccessor(action_observation);

            // If already in the successor list
            if (successor != nullptr)
            {
                // Return the successor node
                return successor->getData();
            }
            else
            {
                // Build next belief and proba
                auto [computed_next_belief, next_belief_probability] = this->computeNextStateAndProbability(belief, action, observation, t);

                // Store the probability of next belief
                this->transition_probability[belief][action][observation] = next_belief_probability;

                // Check if the next belief is already in the graph
                TBelief b = *std::dynamic_pointer_cast<TBelief>(computed_next_belief);
                if (this->state_space_.find(b) == this->state_space_.end())
                {
                    // Add the belief in the space of beliefs
                    this->state_space_.emplace(b, computed_next_belief);
                }

                // Get the next belief
                auto next_belief = this->state_space_.at(b);

                // Add the sucessor in the list of successors
                this->mdp_graph_->getNode(belief)->addSuccessor(action_observation, next_belief);

                return next_belief;
            }
        }
        else
        {
            // Return next belief without storing its value in the graph
            auto [computed_next_belief, proba_belief] = this->computeNextStateAndProbability(belief, action, observation, t);
            return computed_next_belief;
        }
    }

    template <class TBelief>
    std::shared_ptr<State> BaseBeliefMDP<TBelief>::nextState(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t, const std::shared_ptr<HSVI> &hsvi)
    {
        // Select o* as in the paper
        double max_o = -std::numeric_limits<double>::max(), tmp;

        std::shared_ptr<State> selected_next_belief;
        for (const auto &observation : *this->getUnderlyingPOMDP()->getObservationSpace(t))
        {
            const auto &next_belief = this->nextBelief(belief, action, observation->toObservation(), t);
            tmp = this->getObservationProbability(belief, action, nullptr, observation->toObservation(), t) * hsvi->do_excess(next_belief, 0, t + 1);
            if (tmp > max_o)
            {
                max_o = tmp;
                selected_next_belief = next_belief;
            }
        }
        return selected_next_belief;
    }

    template <class TBelief>
    double BaseBeliefMDP<TBelief>::getReward(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t)
    {
        auto state_action = std::make_pair(belief, action);
        auto successor = this->reward_graph_->getNode(0.0)->getSuccessor(state_action);
        if (successor != nullptr)
        {
            // Return the successor node
            return successor->getData();
        }
        else
        {
            // Compute reward : \sum_{s} b(s)r(s,a)
            double reward = 0;
            for (const auto &state : belief->toBelief()->getStates())
            {
                reward += belief->toBelief()->getProbability(state) * this->getUnderlyingProblem()->getReward(state, action, t);
            }
            if ((this->store_states_) && (this->store_action_spaces_ || this->store_actions_))
                this->reward_graph_->getNode(0.0)->addSuccessor(state_action, reward);
            return reward;
        }
    }

    template <class TBelief>
    double BaseBeliefMDP<TBelief>::getObservationProbability(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &, const std::shared_ptr<Observation> &observation, number) const
    {
        return this->transition_probability.at(belief).at(action).at(observation);
    }

    template <class TBelief>
    std::shared_ptr<Space> BaseBeliefMDP<TBelief>::getActionSpaceAt(const std::shared_ptr<State> &, number t)
    {
        return this->getUnderlyingPOMDP()->getActionSpace(t);
    }
    
    // Useless
    template <class TBelief>
    std::shared_ptr<Space> BaseBeliefMDP<TBelief>::getActionSpaceAt(const std::shared_ptr<Observation> &, number t)
    {
        return this->getUnderlyingPOMDP()->getActionSpace(t);
    }

    template <class TBelief>
    std::shared_ptr<Action> BaseBeliefMDP<TBelief>::getRandomAction(const std::shared_ptr<Observation> &, number t)
    {
        return this->getUnderlyingPOMDP()->getActionSpace(t)->sample()->toAction();
    }

    template <class TBelief>
    std::shared_ptr<Observation> BaseBeliefMDP<TBelief>::reset()
    {
        this->step_ = 0;
        this->current_state_ = this->initial_state_;
        std::dynamic_pointer_cast<MDP>(this->getUnderlyingProblem())->reset();
        return this->current_state_;
    }

    template <class TBelief>
    std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> BaseBeliefMDP<TBelief>::step(std::shared_ptr<Action> action)
    {
        auto [observation, rewards, is_done] = this->getUnderlyingProblem()->step(action);
        double belief_reward = this->getReward(this->current_state_, action, this->step_);
        this->current_state_ = this->nextBelief(this->current_state_, action, observation, this->step_);
        this->step_++;
        // if sampled ...
        return std::make_tuple(this->current_state_, std::vector<double>{belief_reward, rewards[0]}, is_done);
    }

    template <class TBelief>
    double BaseBeliefMDP<TBelief>::getExpectedNextValue(const std::shared_ptr<ValueFunction> &value_function, const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t)
    {
        double exp_next_v = 0;
        for (const auto &observation : *this->getUnderlyingPOMDP()->getObservationSpace(t))
        {
            const auto &next_belief = this->nextBelief(belief, action, observation->toObservation(), t);
            exp_next_v += this->getObservationProbability(belief, action, nullptr, observation->toObservation(), t) * value_function->getValueAt(next_belief, t + 1);
        }
        return exp_next_v;
    }

    template <class TBelief>
    std::shared_ptr<POMDPInterface> BaseBeliefMDP<TBelief>::getUnderlyingPOMDP() const
    {
        return std::dynamic_pointer_cast<POMDPInterface>(this->getUnderlyingMDP());
    }

    template <class TBelief>
    std::shared_ptr<Graph<std::shared_ptr<State>, Pair<std::shared_ptr<Action>, std::shared_ptr<Observation>>>> BaseBeliefMDP<TBelief>::getMDPGraph()
    {
        return this->mdp_graph_;
    }

    template <class TBelief>
    std::vector<std::shared_ptr<State>> BaseBeliefMDP<TBelief>::getStoredStates() const
    {
        std::vector<std::shared_ptr<State>> list_states;
        for (const auto &state : this->state_space_)
        {
            list_states.push_back(state.second);
        }
        return list_states;
    }

} // namespace sdm
