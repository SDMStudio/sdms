#include <sdm/world/belief_mdp.hpp>
#include <sdm/core/state/belief_state.hpp>
#include <sdm/core/state/belief_state_graph.hpp>

namespace sdm
{

    BeliefMDP::BeliefMDP()
    {
    }

    BeliefMDP::BeliefMDP(const std::shared_ptr<POMDPInterface> &pomdp, const Belief &initial_belief)
        : SolvableByMDP(pomdp)
    {
        this->initial_state_ = std::make_shared<BeliefStateGraph>(initial_belief);
        this->current_state_ = std::make_shared<BeliefStateGraph>(*std::static_pointer_cast<Belief>(this->initial_state_));
    }

    BeliefMDP::BeliefMDP(const std::shared_ptr<POMDPInterface> &pomdp) : SolvableByMDP(pomdp)
    {
        double proba = 0;
        // std::cout << "Passss ici " << std::endl;
        this->initial_state_ = std::make_shared<BeliefStateGraph>();
        this->current_state_ = std::make_shared<BeliefStateGraph>();

        // Setup initial belief
        for (const auto &state : *pomdp->getStateSpace(0))
        {
            proba = pomdp->getStartDistribution()->getProbability(state->toState(), nullptr);
            if (proba > 0)
            {
                std::static_pointer_cast<BeliefInterface>(this->initial_state_)->setProbability(state->toState(), proba);
            }
        }
        // std::cout << "1 : " << this->initial_state_->str() << std::endl;
        this->current_state_ = this->initial_state_;
        // std::cout << "2" << current_state_->str() << std::endl;
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

    Pair<std::shared_ptr<BeliefInterface>, double> BeliefMDP::nextBelief(const std::shared_ptr<POMDPInterface> &pomdp, const std::shared_ptr<BeliefInterface> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &obs, number t)
    {
        // std::cout << "Pass in nextBelief(b,a,o,t)" << std::endl;
        std::shared_ptr<Belief> next_belief = std::make_shared<Belief>();
        // std::cout << "A" << std::endl;
        double tmp, obs_proba;
        for (const auto &next_state : *pomdp->getStateSpace(t))
        {
            tmp = 0;
            // std::cout << "Z" << std::endl;
            for (const auto &state : *pomdp->getStateSpace(t))
            {
                // std::cout << "E" << std::endl;
                tmp += pomdp->getTransitionProbability(state->toState(), action, next_state->toState(), t) * belief->getProbability(state->toState());
            }
            // std::cout << "R" << std::endl;
            obs_proba = pomdp->getObservationProbability(nullptr, action, next_state->toState(), obs, t);

            // std::cout << "T" << std::endl;
            if (obs_proba && tmp)
            {
                // std::cout << "Y" << std::endl;
                next_belief->setProbability(next_state->toState(), obs_proba * tmp);
            }
        }

        // Compute the coefficient of normalization (eta)
        // std::cout << "Q" << std::endl;
        double eta = next_belief->norm_1();
        // std::cout << "S" << std::endl;
        if (eta > 0)
        {
            // Normalize the belief
            // std::cout << "D" << std::endl;
            for (const auto &state : *pomdp->getStateSpace(t))
            {
                // std::cout << "F" << std::endl;
                next_belief->setProbability(state->toState(), next_belief->getProbability(state->toState()) / eta);
            }
        }
        // std::cout << "G" << std::endl;
        return {next_belief, eta};
    }

    // std::shared_ptr<Distribution<std::shared_ptr<State>>> BeliefMDP::getNextStateDistribution(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t) const
    // {
    //     auto distrib = std::make_shared<DiscreteDistribution<std::shared_ptr<State>>>();
    //     for (const auto &observation : *this->getUnderlyingPOMDP()->getObservationSpace(t))
    //     {
    //         auto [next_belief, proba] = std::static_pointer_cast<BeliefStateGraph>(belief)->next(action, observation->toObservation(), t);
    //         distrib->setProbability(next_belief, proba);
    //     }
    //     return distrib;
    // }

    std::shared_ptr<State> BeliefMDP::nextState(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t) const
    {

        // std::cout << "Pass in nextState(b,a,o,t)" << std::endl;
        return std::static_pointer_cast<BeliefStateGraph>(belief)->next(BeliefMDP::nextBelief, this->getUnderlyingPOMDP(), action, observation, t);
    }

    std::shared_ptr<State> BeliefMDP::nextState(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t, const std::shared_ptr<HSVI> &hsvi) const
    {
        // std::cout << "Pass in nextState" << std::endl;
        // Select o* as in the paper
        double max_o = -std::numeric_limits<double>::max(), tmp;
        std::shared_ptr<State> selected_next_belief;

        for (const auto &observation : *this->getUnderlyingPOMDP()->getObservationSpace(t))
        {
            // std::cout << "observation=" << observation->str() << std::endl;
            const auto &next_belief = this->nextState(std::static_pointer_cast<BeliefInterface>(belief), action, observation->toObservation(), t);
            // std::cout << "next_belief=" << next_belief->str() << std::endl;
            tmp = this->getObservationProbability(belief, action, nullptr, observation->toObservation(), t) * hsvi->do_excess(next_belief, 0, t + 1);
            // std::cout << "tmp=" << tmp << std::endl;
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
        // std::cout << "R(" << *belief << ", " << *action << ", " << t << ") = " << std::endl;
        // Compute reward : \sum_{s} b(s)r(s,a)
        double reward = 0;
        for (const auto &state : *this->getUnderlyingProblem()->getStateSpace(t))
        {
            reward += std::static_pointer_cast<BeliefInterface>(belief)->getProbability(state->toState()) * this->getUnderlyingProblem()->getReward(state->toState(), action, t);
        }
        // std::cout << reward << std::endl;
        return reward;
    }

    double BeliefMDP::getObservationProbability(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &, const std::shared_ptr<Observation> &observation, number) const
    {
        return std::static_pointer_cast<BeliefStateGraph>(belief)->getProbability(action, observation);
        // double proba = 0, tmp;

        // for (const auto &state : *this->getUnderlyingProblem()->getStateSpace(t))
        // {
        //     tmp = 0;
        //     for (const auto &next_state : this->getUnderlyingProblem()->getReachableStates(state->toState(), action, t))
        //     {
        //         tmp += this->getUnderlyingPOMDP()->getDynamics(state->toState(), action, next_state->toState(), observation, t);
        //     }
        //     proba += tmp *  std::static_pointer_cast<BeliefInterface>(belief)->getProbability(state->toState());
        // }
        // return proba;
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
