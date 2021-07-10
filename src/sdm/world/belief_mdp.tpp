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
    BaseBeliefMDP<TBelief>::BaseBeliefMDP(const std::shared_ptr<POMDPInterface> &pomdp) : SolvableByMDP(pomdp)
    {
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

        this->initial_state_ = initial_state;
        this->mdp_graph_ = std::make_shared<Graph<std::shared_ptr<State>, Pair<std::shared_ptr<Action>, std::shared_ptr<Observation>>>>();
        this->mdp_graph_->addNode(this->initial_state_);
    }

    template <class TBelief>
    Pair<std::shared_ptr<State>, double> BaseBeliefMDP<TBelief>::computeNextStateAndProbability(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        std::shared_ptr<TBelief> next_belief = std::make_shared<TBelief>();

        for (const auto &next_state : *this->getUnderlyingPOMDP()->getStateSpace(t + 1))
        {
            double next_state_probability = 0;
            for (const auto &state : *this->getUnderlyingPOMDP()->getStateSpace(t))
            {
                next_state_probability += this->getUnderlyingPOMDP()->getDynamics(state->toState(), action, next_state->toState(), observation, t) * belief->toBelief()->getProbability(state->toState());
            }
            if (next_state_probability > 0)
            {
                next_belief->setProbability(next_state->toState(), next_state_probability);
            }
        }

        // Compute the coefficient of normalization (eta)
        double eta = next_belief->norm_1();
        if (eta > 0)
        {
            // Normalize the belief
            for (const auto &state : next_belief->getStates())
            {
                next_belief->setProbability(state->toState(), next_belief->getProbability(state->toState()) / eta);
            }
        }
        return {next_belief, eta};
    }

    template <class TBelief>
    std::shared_ptr<State> BaseBeliefMDP<TBelief>::nextBelief(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        auto action_observation = std::make_pair(action, observation);

        if (this->backup)
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
                auto iterator_on_belief = this->state_space_.find(b);
                if (iterator_on_belief == this->state_space_.end())
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
            auto [computed_next_belief, next_belief_probability] = this->computeNextStateAndProbability(belief, action, observation, t);
            // Check if the next belief is already in the graph
            TBelief b = *std::dynamic_pointer_cast<TBelief>(computed_next_belief);
            auto iterator_on_belief = this->state_space_.find(b);
            if (iterator_on_belief == this->state_space_.end())
            {
                // Add the belief in the space of beliefs
                this->state_space_.emplace(b, computed_next_belief);
            }

            // Get the next belief
            auto next_belief = this->state_space_.at(b);
            return next_belief;
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
        // Compute reward : \sum_{s} b(s)r(s,a)
        double reward = 0;
        for (const auto &state : belief->toBelief()->getStates())
        {
            reward += belief->toBelief()->getProbability(state) * this->getUnderlyingProblem()->getReward(state, action, t);
        }
        return reward;
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

    template <class TBelief>
    std::shared_ptr<Space> BaseBeliefMDP<TBelief>::getActionSpaceAt(const std::shared_ptr<Observation> &, number t)
    {
        return this->getUnderlyingPOMDP()->getActionSpace(t);
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
        std::tie(this->next_observation_, this->rewards_, this->is_done_) = std::dynamic_pointer_cast<MDP>(this->getUnderlyingProblem())->step(action);
        double reward = this->getReward(this->current_state_, action, this->step_);
        this->current_state_ = this->nextBelief(this->current_state_, action, this->next_observation_, this->step_);
        this->step_++;
        return std::make_tuple(this->current_state_, std::vector<double>{reward, this->rewards_[0]}, this->is_done_);
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

} // namespace sdm
