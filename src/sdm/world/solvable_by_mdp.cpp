#include <sdm/world/solvable_by_mdp.hpp>
#include <sdm/world/mdp.hpp>
#include <sdm/core/observation/default_observation.hpp>

namespace sdm
{

    SolvableByMDP::SolvableByMDP()
    {
    }

    SolvableByMDP::SolvableByMDP(const std::shared_ptr<MDPInterface> &mdp) : underlying_problem_(mdp)
    {
        this->initial_state_ = (*this->getUnderlyingProblem()->getStateSpace(0)->begin())->toState();
    }

    void SolvableByMDP::setInitialState(const std::shared_ptr<State> &state)
    {
        this->initial_state_ = state;
    }

    std::shared_ptr<State> SolvableByMDP::getInitialState()
    {
        return this->initial_state_;
    }

    number SolvableByMDP::getHorizon() const
    {
        return this->getUnderlyingProblem()->getHorizon();
    }

    std::shared_ptr<Distribution<std::shared_ptr<State>>> SolvableByMDP::getStartDistribution() const
    {
        return this->getUnderlyingMDP()->getStartDistribution();
    }

    // std::shared_ptr<State> SolvableByMDP::nextState(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t, const std::shared_ptr<HSVI> &hsvi)
    // {
    //     double max = -std::numeric_limits<double>::max();
    //     std::shared_ptr<State> argmax = 0;

    //     auto observation_space = this->getObservationSpaceAt(state,action,t);
    //     for (const auto &next_state : *observation_space)
    //     {
    //         double tmp = this->underlying_problem_->getTransitionProbability(state, action, next_state->toState(), t) * hsvi->do_excess(next_state->toState(), 0, t + 1);
    //         if (tmp > max)
    //         {
    //             max = tmp;
    //             argmax = next_state->toState();
    //         }
    //     }
    //     return argmax;
    // }

    std::shared_ptr<Space> SolvableByMDP::getActionSpaceAt(const std::shared_ptr<State> &, number t)
    {
        return std::dynamic_pointer_cast<MDPInterface>(this->underlying_problem_)->getActionSpace(t);
    }

    double SolvableByMDP::getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        return this->underlying_problem_->getReward(state, action, t);
    }

    double SolvableByMDP::getExpectedNextValue(const std::shared_ptr<ValueFunction> &value_function, const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        double tmp = 0.0;

        for (const auto &next_state : this->underlying_problem_->getReachableStates(state, action, t))
        {
            tmp += this->underlying_problem_->getTransitionProbability(state, action, next_state, t) * value_function->getValueAt(next_state, t + 1);
        }
        return tmp;
    }

    const std::shared_ptr<MDPInterface> &SolvableByMDP::getUnderlyingProblem() const
    {
        return this->underlying_problem_;
    }

    const std::shared_ptr<MDPInterface> &SolvableByMDP::getUnderlyingMDP() const
    {
        return this->underlying_problem_;
    }

    double SolvableByMDP::getDiscount(number t) const
    {
        return this->underlying_problem_->getDiscount(t);
    }

    double SolvableByMDP::getWeightedDiscount(number t)
    {
        return std::pow(this->getDiscount(t), t);
    }

    double SolvableByMDP::do_excess(double, double lb, double ub, double, double error, number t)
    {
        return (ub - lb) - error / this->getWeightedDiscount(t);
    }

    Pair<std::shared_ptr<Action>, double> SolvableByMDP::selectNextAction(const std::shared_ptr<ValueFunction> &, const std::shared_ptr<ValueFunction> &ub, const std::shared_ptr<State> &s, number h)
    {
        return ub->getGreedyActionAndValue(s, h);
    }

    std::shared_ptr<Space> SolvableByMDP::getObservationSpaceAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        auto reachable_set = this->underlying_problem_->getReachableStates(state, action, t);
        return std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<State>>(reachable_set.begin(), reachable_set.end()));
    }

    Pair<std::shared_ptr<State>, double> SolvableByMDP::getNextState(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        return std::make_pair(observation->toState(), this->underlying_problem_->getTransitionProbability(state, action, observation->toState(), t));
    }

    Pair<std::shared_ptr<State>, double> SolvableByMDP::getNextStateAndProba(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        return std::make_pair(observation->toState(), this->getUnderlyingProblem()->getTransitionProbability(state, action, observation->toState(), t));
    }

}
