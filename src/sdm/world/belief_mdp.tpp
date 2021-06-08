#include <sdm/world/belief_mdp.hpp>

namespace sdm
{

    BeliefMDP::BeliefMDP()
    {
    }

    BeliefMDP::BeliefMDP(const std::shared_ptr<POMDPInterface> &pomdp) : SolvableByMDP(pomdp)
    {
        double proba = 0;
        for (const auto &state : *pomdp->getStateSpace(0))
        {
            proba = pomdp->getStartDistribution()->getProbability(std::static_pointer_cast<State>(state), nullptr);
            if (proba > 0)
            {
                this->initial_state_->setProbability(std::static_pointer_cast<State>(state), proba);
            }
        }
        this->current_state_ = this->initial_state_;
    }

    // std::shared_ptr<Observation> BeliefMDP::reset()
    // {
    //     this->current_state_ = this->initial_state_;
    //     this->getUnderlyingProblem()->reset();
    //     return this->current_state_;
    // }

    // std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> BeliefMDP::step(std::shared_ptr<Action> action)
    // {
    //     auto [next_obs, rewards, done] = this->getUnderlyingProblem()->step(action);
    //     this->current_state_ = this->nextState(this->current_state_, action, next_obs);
    //     return std::make_tuple(this->current_state_, rewards, done);
    // }

    std::shared_ptr<BeliefInterface> BeliefMDP::nextState(const std::shared_ptr<BeliefInterface> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &obs, number t) const
    {
        std::shared_ptr<BeliefInterface> next_belief;
        double tmp, obs_proba;
        for (const auto &next_state : *this->getUnderlyingProblem()->getStateSpace(t))
        {
            tmp = 0;
            for (const auto &state : *this->getUnderlyingProblem()->getStateSpace(t))
            {
                tmp += this->getUnderlyingProblem()->getTransitionProbability(std::static_pointer_cast<State>(state), action, std::static_pointer_cast<State>(next_state), t) * belief->getProbability(std::static_pointer_cast<State>(state));
            }
            obs_proba = this->getUnderlyingPOMDP()->getObservationProbability(nullptr, action, std::static_pointer_cast<State>(next_state), obs, t);

            if (obs_proba && tmp)
            {
                next_belief->setProbability(std::static_pointer_cast<State>(next_state), obs_proba * tmp);
            }
        }
        // Normalize the belief
        double sum = next_belief.norm_1();
        for (const auto &state : *this->getUnderlyingProblem()->getStateSpace(t))
        {
            next_belief->setProbability(std::static_pointer_cast<State>(state), next_belief->getProbability(std::static_pointer_cast<State>(state)) / sum);
        }
        return next_belief;
    }

    std::shared_ptr<State> BeliefMDP::nextState(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t, std::shared_ptr<HSVI> hsvi) const
    {
        // Select o* as in the paper
        double max_o = -std::numeric_limits<double>::max(), tmp;
        std::shared_ptr<BeliefInterface> select_next_state;
        for (const auto &observation : *this->getUnderlyingPOMDP()->getObservationSpace(t))
        {
            const auto &next_belief = this->nextState(std::static_pointer_cast<BeliefInterface>(belief), action, std::static_pointer_cast<Observation>(observation), t);
            tmp = this->getObservationProbability(belief, action, next_belief, std::static_pointer_cast<Observation>(observation), t) * hsvi->do_excess(next_belief, 0, t + 1);
            if (tmp > max_o)
            {
                max_o = tmp;
                select_next_state = next_belief;
            }
        }
        return select_next_state;
    }

    double BeliefMDP::getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    {
        std::shared_ptr<BeliefInterface> belief = std::static_pointer_cast<BeliefInterface>(state);
        double reward = 0;
        for (const auto &state : *this->getUnderlyingProblem()->getStateSpace(t))
        {
            reward += belief->getProbability(std::static_pointer_cast<State>(state)) * this->getUnderlyingProblem()->getReward(std::static_pointer_cast<State>(state), action, t);
        }
        return reward;
    }

    double BeliefMDP::getObservationProbability(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_belief, const std::shared_ptr<Observation> &observation, number t) const
    {
        double proba = 0, tmp;
        for (const auto &state : *this->getUnderlyingProblem()->getStateSpace(t))
        {
            tmp = 0;
            for (const auto &next_state : this->getUnderlyingProblem()->getReachableStates(std::static_pointer_cast<State>(state), action, t))
            {
                tmp += this->getUnderlyingPOMDP()->getDynamics(std::static_pointer_cast<State>(state), action, std::static_pointer_cast<State>(next_state), observation, t);
            }
            proba += tmp * std::static_pointer_cast<BeliefInterface>(belief)->getProbability(std::static_pointer_cast<State>(state));
        }
        return proba;
    }

    std::shared_ptr<State> BeliefMDP::getInitialState()
    {
        return this->initial_state_;
    }

    std::shared_ptr<Space> BeliefMDP::getActionSpaceAt(const std::shared_ptr<State> &)
    {
        return this->getUnderlyingPOMDP()->getActionSpace(0);
    }

    double BeliefMDP::getExpectedNextValue(std::shared_ptr<ValueFunction> value_function, const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t) const
    {
        double exp_next_v = 0;
        for (const auto &observation : *this->getUnderlyingPOMDP()->getObservationSpace(t))
        {
            const auto &next_belief = this->nextState(std::static_pointer_cast<BeliefInterface>(belief), action, std::static_pointer_cast<Observation>(observation), t);
            exp_next_v += this->getObservationProbability(std::static_pointer_cast<BeliefInterface>(belief), action, std::static_pointer_cast<BeliefInterface>(next_belief), std::static_pointer_cast<Observation>(observation), t) * value_function->getValueAt(std::static_pointer_cast<BeliefInterface>(next_belief), t + 1);
        }
        return exp_next_v;
    }

    std::shared_ptr<POMDPInterface> BeliefMDP::getUnderlyingPOMDP() const
    {
        return std::static_pointer_cast<POMDPInterface>(this->getUnderlyingMDP());
    }

} // namespace sdm
