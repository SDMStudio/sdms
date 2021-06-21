#include <sdm/world/belief_mdp.hpp>
#include <sdm/core/state/belief_state.hpp>

namespace sdm
{

    BeliefMDP::BeliefMDP()
    {
    }

    BeliefMDP::BeliefMDP(const std::shared_ptr<POMDPInterface> &pomdp) : SolvableByMDP(pomdp)
    {
        double proba = 0;
        auto initial_state = std::make_shared<Belief>();

        for (const auto &state : *pomdp->getStateSpace(0))
        {
            proba = pomdp->getStartDistribution()->getProbability(state->toState(), nullptr);
            if (proba > 0)
            {
                initial_state->setProbability(state->toState(), proba);
            }
        }
        this->initial_state_ = initial_state;
        *this->current_state_ = *this->initial_state_;
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
        std::shared_ptr<BeliefInterface> next_belief = std::make_shared<Belief>();
        double tmp, obs_proba;
        for (const auto &next_state : *this->getUnderlyingProblem()->getStateSpace(t))
        {
            tmp = 0;
            for (const auto &state : *this->getUnderlyingProblem()->getStateSpace(t))
            {
                tmp += this->getUnderlyingProblem()->getTransitionProbability(state->toState(), action, next_state->toState(), t) * belief->getProbability(state->toState());
            }
            obs_proba = this->getUnderlyingPOMDP()->getObservationProbability(nullptr, action, next_state->toState(), obs, t);

            if (obs_proba && tmp)
            {
                next_belief->setProbability(next_state->toState(), obs_proba * tmp);
            }
        }

        // Normalize the belief
        double sum = std::dynamic_pointer_cast<Belief>(next_belief)->norm_1();
        if(sum>0)
        {
            for (const auto &state : *this->getUnderlyingProblem()->getStateSpace(t))
            {
                next_belief->setProbability(state->toState(), next_belief->getProbability(state->toState()) / sum);
            }
        }
        return next_belief;
    }

    std::shared_ptr<State> BeliefMDP::nextState(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t, const std::shared_ptr<HSVI> &hsvi) const
    {
        // Select o* as in the paper
        double max_o = -std::numeric_limits<double>::max(), tmp;
        std::shared_ptr<BeliefInterface> select_next_state;
        // std::cout<<"****** Current belief"<<belief->str()<<std::endl;

        for (const auto &observation : *this->getUnderlyingPOMDP()->getObservationSpace(t))
        {
            const auto &next_belief = this->nextState(belief->toBelief(), action, observation->toObservation(), t);
            tmp = this->getObservationProbability(belief, action, nullptr, observation->toObservation(), t) * hsvi->do_excess(next_belief, 0, t + 1);
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
            reward += belief->getProbability(state->toState()) * this->getUnderlyingProblem()->getReward(state->toState(), action, t);
        }
        return reward;
    }

    double BeliefMDP::getObservationProbability(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &, const std::shared_ptr<Observation> &observation, number t) const
    {
        double proba = 0, tmp;

        for (const auto &state : *this->getUnderlyingProblem()->getStateSpace(t))
        {
            tmp = 0;
            for (const auto &next_state : this->getUnderlyingProblem()->getReachableStates(state->toState(), action, t))
            {
                tmp += this->getUnderlyingPOMDP()->getDynamics(state->toState(), action, next_state->toState(), observation, t);
            }
            proba += tmp * belief->toBelief()->getProbability(state->toState());
        }
        return proba;
    }

    // std::shared_ptr<State> BeliefMDP::getInitialState()
    // {
    //     return this->initial_state_;
    // }

    std::shared_ptr<Space> BeliefMDP::getActionSpaceAt(const std::shared_ptr<State> &, number t)
    {
        return this->getUnderlyingPOMDP()->getActionSpace(t);
    }

    double BeliefMDP::getExpectedNextValue(const std::shared_ptr<ValueFunction> &value_function, const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t) const
    {
        double exp_next_v = 0;
        for (const auto &observation : *this->getUnderlyingPOMDP()->getObservationSpace(t))
        {
            const auto &next_belief = this->nextState(belief->toBelief(), action, observation->toObservation(), t);
            exp_next_v += this->getObservationProbability(belief->toBelief(), action, nullptr, observation->toObservation(), t) * value_function->getValueAt(next_belief->toBelief(), t + 1);
        }
        return exp_next_v;
    }

    std::shared_ptr<POMDPInterface> BeliefMDP::getUnderlyingPOMDP() const
    {
        return std::dynamic_pointer_cast<POMDPInterface>(this->getUnderlyingMDP());
    }

} // namespace sdm
