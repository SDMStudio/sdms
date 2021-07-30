#include <sdm/core/state/belief_state.hpp>
#include <sdm/utils/struct/graph.hpp>

namespace sdm
{

    template <class TBelief>
    BaseBeliefMDP<TBelief>::BaseBeliefMDP()
    {
    }

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
        this->state_space_[*initial_state] = this->initial_state_;

        this->reward_graph_ = std::make_shared<Graph<double, Pair<std::shared_ptr<State>, std::shared_ptr<Action>>>>();
        this->reward_graph_->addNode(0.0);
    }

    template <class TBelief>
    std::shared_ptr<Space> BaseBeliefMDP<TBelief>::getObservationSpace(number t)
    {
        return this->getUnderlyingPOMDP()->getObservationSpace(t);
    }

    template <class TBelief>
    std::shared_ptr<Space> BaseBeliefMDP<TBelief>::getActionSpaceAt(const std::shared_ptr<State> &, number t)
    {
        return this->getUnderlyingPOMDP()->getActionSpace(t);
    }

    template <class TBelief>
    std::shared_ptr<POMDPInterface> BaseBeliefMDP<TBelief>::getUnderlyingPOMDP() const
    {
        return std::dynamic_pointer_cast<POMDPInterface>(this->getUnderlyingMDP());
    }

    template <class TBelief>
    Pair<std::shared_ptr<State>, double> BaseBeliefMDP<TBelief>::computeNextStateAndProbability(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        // Compute next state
        std::shared_ptr<State> next_belief = this->computeNextState(belief, action, observation, t);
        // Compute the coefficient of normalization (eta)
        double eta = next_belief->toBelief()->norm_1();
        // Normalize to belief
        next_belief->toBelief()->normalizeBelief(eta);
        // Return the pair next belief / proba of the transition in this belief
        return {next_belief->toBelief(), eta};
    }

    template <class TBelief>
    std::shared_ptr<State> BaseBeliefMDP<TBelief>::computeNextState(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
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

    // ------------------------------------------------------
    // FONCTIONS COMMON TO ALL BELIEFMDP / OCCUPANCYMDP
    // ------------------------------------------------------

    template <class TBelief>
    Pair<std::shared_ptr<State>, double> BaseBeliefMDP<TBelief>::nextBeliefAndProba(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        auto action_observation = std::make_pair(action, observation);

        // If we store data in the graph
        if (this->store_states_ && this->store_actions_)
        {

            // Get the successor
            auto successor = this->mdp_graph_->getNode(belief)->getSuccessor(action_observation);

            // If already in the successor list
            if (successor != nullptr)
            {
                // Return the successor node
                return {successor->getData(), this->transition_probability.at(belief).at(action).at(observation)};
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

                return {next_belief, next_belief_probability};
            }
        }
        else if (this->store_states_)
        {
            // Return next belief without storing its value in the graph
            auto [computed_next_belief, proba_belief] = this->computeNextStateAndProbability(belief, action, observation, t);
            TBelief b = *std::dynamic_pointer_cast<TBelief>(computed_next_belief);
            if (this->state_space_.find(b) == this->state_space_.end())
            {
                // Add the belief in the space of beliefs
                this->state_space_.emplace(b, computed_next_belief);
            }
            return {this->state_space_.at(b), proba_belief};
        }
        else
        {
            // Return next belief without storing its value in the graph
            return this->computeNextStateAndProbability(belief, action, observation, t);
        }
    }

    template <class TBelief>
    std::shared_ptr<State> BaseBeliefMDP<TBelief>::nextBelief(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        return this->nextBeliefAndProba(belief, action, observation, t).first;
    }

    template <class TBelief>
    std::shared_ptr<State> BaseBeliefMDP<TBelief>::nextState(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t, const std::shared_ptr<HSVI> &hsvi)
    {
        // Select o* as in the paper
        double max_o = -std::numeric_limits<double>::max(), tmp;

        std::shared_ptr<State> selected_next_belief;
        auto observation_space = this->getObservationSpace(t);
        for (const auto &observation : *observation_space)
        {
            // Get the next state and probability
            auto [next_belief, belief_transition_proba] = this->nextBeliefAndProba(belief, action, observation->toObservation(), t);
            // Compute error correlated to this next belief
            tmp = belief_transition_proba * hsvi->do_excess(next_belief, 0, t + 1);
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
            if (this->store_states_ && this->store_actions_)
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
    double BaseBeliefMDP<TBelief>::getExpectedNextValue(const std::shared_ptr<ValueFunction> &value_function, const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t)
    {

        double exp_next_v = 0;
        // For all observations from the controller point of view
        auto accessible_observation_space = this->getObservationSpace(t);
        for (const auto &observation : *accessible_observation_space)
        {
            // Check if we can skip the computation of the next occupancy state.
            // -> if the timestep is greater than the current horizon
            bool skip_compute_next_state = (value_function->isFiniteHorizon() && ((t + 1) >= value_function->getHorizon()));
            // Compute next state (if required)
            auto [next_state, state_transition_proba] = (skip_compute_next_state) ? Pair<std::shared_ptr<State>, double>({nullptr, 1.}) : this->nextBeliefAndProba(belief, action, observation->toObservation(), t);
            // Update the next expected value at the next state
            exp_next_v += state_transition_proba * value_function->getValueAt(next_state, t + 1);
        }
        return exp_next_v;
    }

    // ------------------------------------------------------
    // FONCTIONS REQUIRED IN LEARNING ALGORITHMS
    // ------------------------------------------------------

    template <class TBelief>
    std::shared_ptr<Observation> BaseBeliefMDP<TBelief>::reset()
    {
        this->step_ = 0;
        this->current_state_ = this->initial_state_;
        std::dynamic_pointer_cast<GymInterface>(this->getUnderlyingProblem())->reset();
        return this->current_state_;
    }

    template <class TBelief>
    std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> BaseBeliefMDP<TBelief>::step(std::shared_ptr<Action> action)
    {
        // Do a step on the underlying problem
        auto [observation, rewards, is_done] = this->getUnderlyingProblem()->step(action);
        // Compute reward
        double belief_reward = this->getReward(this->current_state_, action, this->step_);
        // Compute next belief
        this->current_state_ = this->nextBelief(this->current_state_, action, observation, this->step_);
        this->step_++;
        // if sampled ...
        return std::make_tuple(this->current_state_, std::vector<double>{belief_reward, rewards[0]}, is_done);
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

    // ------------------------------------------------------
    // ACCESSORS OF SOME SPECIAL DATA COMMON TO ALL BELIEF MDP
    // ------------------------------------------------------

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
