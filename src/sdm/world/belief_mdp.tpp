#include <sdm/world/belief_mdp.hpp>

namespace sdm
{

    BeliefMDP::BeliefMDP()
    {
    }

    BeliefMDP::BeliefMDP(std::shared_ptr<BasePOMDP> pomdp) : pomdp_(pomdp) {}

    BeliefMDP::BeliefMDP(std::string underlying_dpomdp) : BeliefMDP(std::make_shared<DiscretePOMDP>(underlying_dpomdp))
    {
    }

    std::shared_ptr<State> BeliefMDP::reset()
    {
        this->current_state_ = this->initial_state_;
        this->pomdp_->reset();
        return this->current_state_;
    }

    std::tuple<std::shared_ptr<State>, std::vector<double>, bool> BeliefMDP::step(std::shared_ptr<Action> action)
    {
        auto [next_obs, rewards, done] = this->pomdp_->step(action);
        this->current_state_ = this->nextState(this->current_state_, action, next_obs);
        return std::make_tuple(this->current_state_, rewards, done);
    }

    std::shared_ptr<State> BeliefMDP::nextState(const std::shared_ptr<BeliefState> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &obs, number t) const
    {
        std::shared_ptr<BeliefState> next_belief;
        double tmp, obs_proba;
        for (const auto &next_state : this->pomdp_->getAllStates(t))
        {
            tmp = 0;
            for (const auto &state : this->pomdp_->getAllStates(t))
            {
                tmp += this->pomdp_->getTransitionProbability(state, action, next_state, t) * belief->getProbabilityAt(state);
            }
            obs_proba = this->pomdp_->getObservationProbability(next_state, action, obs, next_state, t);

            if (obs_proba && tmp)
            {
                next_belief->setProbabilityAt(next_state, obs_proba * tmp);
            }
        }
        // Normalize the belief
        double sum = next_belief.norm_1();
        for (const auto &state : this->pomdp_->getAllStates(t))
        {
            next_belief->setProbabilityAt(state, new_belief->getProbabilityAt(state) / sum);
        }
        return next_belief;
    }

    std::shared_ptr<State> BeliefMDP::nextState(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t, std::shared_ptr<HSVI<std::shared_ptr<State>, std::shared_ptr<Action>>> hsvi) const
    {
        // Select o* as in the paper
        double max_o = -std::numeric_limits<double>::max(), tmp;
        std::shared_ptr<BeliefState> select_next_state;
        for (const auto &observation : this->pomdp_->getAllObservations(t))
        {
            const auto &tau = this->nextState(belief, action, o);
            tmp = this->getObservationProbability(belief, action, observation, t) * hsvi->do_excess(tau, 0, t + 1);
            if (tmp > max_o)
            {
                max_o = tmp;
                select_next_state = tau;
            }
        }
        return select_next_state;
    }

    double BeliefMDP::getReward(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t) const
    {
        double reward = 0;
        for (const auto &state : this->pomdp_->getAllStates(t))
        {
            reward += belief->getProbabilityAt(state) * this->pomdp_->getReward(state, action, t);
        }
        return reward;
    }

    double BeliefMDP::getObservationProbability(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t) const
    {
        double proba = 0, tmp;
        for (const auto &state : this->pomdp_->getAllStates(t))
        {
            tmp = 0;
            for (const auto &next_state : this->pomdp_->getReachableStates(state, action, t))
            {
                tmp += this->pomdp_->getDynamics(state, action, observation, next_state, t);
            }
            proba += tmp * belief->getProbabilityAt(s);
        }
        return proba;
    }

    bool BeliefMDP::isSerialized() const
    {
        return false;
    }

    DiscretePOMDP *BeliefMDP::getUnderlyingProblem()
    {
        return this->pomdp_.get();
    }

    std::shared_ptr<State> BeliefMDP::getInitialState()
    {
        return this->initial_state_;
    }

    std::shared_ptr<DiscreteSpace<std::shared_ptr<Action>>> BeliefMDP::getActionSpaceAt(const std::shared_ptr<State> &)
    {
        return this->pomdp_->getActionSpace();
    }

    double BeliefMDP::getExpectedNextValue(std::shared_ptr<ValueFunction<std::shared_ptr<State>, std::shared_ptr<Action>>> value_function, const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t) const
    {
        double exp_next_v = 0;
        for (const auto &observation : this->pomdp_->getAllObservations(t))
        {
            const auto &next_belief = this->nextState(belief, action, observation);
            exp_next_v += this->getObservationProbability(belief, action, observation, t) * value_function->getValueAt(next_belief, t + 1);
        }
        return exp_next_v;
    }

    double BeliefMDP::getDiscount(number t)
    {
        return this->pomdp_->getDiscount(t);
    }

    double BeliefMDP::getWeightedDiscount(number horizon)
    {
        return std::pow(this->pomdp_->getDiscount(), horizon);
    }

    double BeliefMDP::do_excess(double, double lb, double ub, double, double error, number horizon)
    {
        return (ub - lb) - error / this->getWeightedDiscount(horizon);
    }

    std::shared_ptr<Action> BeliefMDP::selectNextAction(const std::shared_ptr<ValueFunction> &, const std::shared_ptr<ValueFunction> &ub, const std::shared_ptr<State> &s, number h)
    {
        return ub->getBestAction(s, h);
    }

} // namespace sdm
