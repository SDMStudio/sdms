#include <sdm/world/belief_mdp.hpp>
#include <sdm/core/state/belief_state.hpp>
#include <sdm/core/state/belief_state_graph.hpp>

namespace sdm
{

    BeliefMDP::BeliefMDP()
    {
    }

    // BeliefMDP::BeliefMDP(const std::shared_ptr<POMDPInterface> &pomdp, const Belief &initial_belief)
    //     : SolvableByMDP(pomdp)
    // {
    //     this->initial_state_ = std::make_shared<BeliefStateGraph>(initial_belief);
    //     this->current_state_ = std::make_shared<BeliefStateGraph>(*std::static_pointer_cast<Belief>(this->initial_state_));
    // }

    BeliefMDP::BeliefMDP(const std::shared_ptr<POMDPInterface> &pomdp) : SolvableByMDP(pomdp)
    {
        this->initial_state_ = std::make_shared<BeliefStateGraph>();

        // For each state at t=0:
        for (const auto &state : *pomdp->getStateSpace(0))
        {
            // Get the state's probability
            double probability = pomdp->getStartDistribution()->getProbability(state->toState(), nullptr);
            // If the state is possible:
            if (probability > 0)
            {
                // Set the probability
                std::static_pointer_cast<BeliefInterface>(this->initial_state_)->setProbability(state->toState(), probability);
            }
        }

        std::static_pointer_cast<BeliefStateGraph>(this->initial_state_)->initialize();
        this->current_state_ = this->initial_state_;
    }

    Pair<std::shared_ptr<BeliefInterface>, double> BeliefMDP::nextBelief(const std::shared_ptr<POMDPInterface> &pomdp, const std::shared_ptr<BeliefInterface> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        std::shared_ptr<Belief> next_belief = std::make_shared<Belief>();

        // For each state at t+1:
        for (const auto &next_state : *pomdp->getStateSpace(t + 1))
        {
            double probability = 0;
            for (const auto &state : *pomdp->getStateSpace(t))
            {
                probability += pomdp->getDynamics(state->toState(), action, next_state->toState(), observation, t) * belief->getProbability(state->toState());
            }
            //  If the state is possible:
            if (probability > 0)
            {
                next_belief->setProbability(next_state->toState(), probability);
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

    std::shared_ptr<State> BeliefMDP::nextState(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t) const
    {
        // Get next belief from the belief graph, given a belief transition function
        return std::static_pointer_cast<BeliefStateGraph>(belief)->next(BeliefMDP::nextBelief, this->getUnderlyingPOMDP(), action, observation, t);
    }

    std::shared_ptr<State> BeliefMDP::nextState(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t, const std::shared_ptr<HSVI> &hsvi) const
    {
        // Select o* as in the paper
        double max_o = -std::numeric_limits<double>::max(), tmp;

        std::shared_ptr<State> selected_next_belief;
        for (const auto &observation : *this->getUnderlyingPOMDP()->getObservationSpace(t))
        {
            const auto &next_belief = this->nextState(belief, action, observation->toObservation(), t);
            tmp = this->getObservationProbability(belief, action, nullptr, observation->toObservation(), t) * hsvi->do_excess(next_belief, 0, t + 1);
            if (tmp > max_o)
            {
                max_o = tmp;
                selected_next_belief = next_belief;
            }
        }
        return selected_next_belief;
    }

    double BeliefMDP::getReward(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t) const
    {
        // Compute reward : \sum_{s} b(s)r(s,a)
        double reward = 0;
        for (const auto &state : *this->getUnderlyingProblem()->getStateSpace(t))
        {
            reward += std::static_pointer_cast<BeliefInterface>(belief)->getProbability(state->toState()) * this->getUnderlyingProblem()->getReward(state->toState(), action, t);
        }
        return reward;
    }

    double BeliefMDP::getObservationProbability(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &, const std::shared_ptr<Observation> &observation, number) const
    {
        return std::static_pointer_cast<BeliefStateGraph>(belief)->getProbability(action, observation);
    }

    std::shared_ptr<Space> BeliefMDP::getActionSpaceAt(const std::shared_ptr<State> &, number t)
    {
        return this->getUnderlyingPOMDP()->getActionSpace(t);
    }

    std::shared_ptr<Space> BeliefMDP::getActionSpaceAt(const std::shared_ptr<Observation> &, number t)
    {
        return this->getUnderlyingPOMDP()->getActionSpace(t);
    }

    std::shared_ptr<Observation> BeliefMDP::reset()
    {
        this->step_ = 0;
        this->current_state_ = this->initial_state_;
        std::dynamic_pointer_cast<MDP>(this->getUnderlyingProblem())->reset();
        return this->current_state_;
    }

    std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> BeliefMDP::step(std::shared_ptr<Action> action)
    {
        // auto [next_obs, rewards, done] = std::dynamic_pointer_cast<MDP>(this->getUnderlyingProblem())->step(action);
        auto feedback = std::dynamic_pointer_cast<MDP>(this->getUnderlyingProblem())->step(action);
        auto next_obs = std::get<0>(feedback);
        this->current_state_ = this->nextState(this->current_state_, action, next_obs, this->step_);
        return std::make_tuple(this->current_state_, std::get<1>(feedback), std::get<2>(feedback));

    }

    double BeliefMDP::getExpectedNextValue(const std::shared_ptr<ValueFunction> &value_function, const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t) const
    {
        double exp_next_v = 0;
        for (const auto &observation : *this->getUnderlyingPOMDP()->getObservationSpace(t))
        {
            const auto &next_belief = this->nextState(belief, action, observation->toObservation(), t);
            exp_next_v += this->getObservationProbability(belief, action, nullptr, observation->toObservation(), t) * value_function->getValueAt(next_belief, t + 1);
        }
        return exp_next_v;
    }

    std::shared_ptr<POMDPInterface> BeliefMDP::getUnderlyingPOMDP() const
    {
        return std::dynamic_pointer_cast<POMDPInterface>(this->getUnderlyingMDP());
    }

} // namespace sdm
