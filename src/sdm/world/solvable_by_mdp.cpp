#include <sdm/world/solvable_by_mdp.hpp>
#include <sdm/world/mdp.hpp>

namespace sdm
{

    SolvableByMDP::SolvableByMDP()
    {
    }

    SolvableByMDP::SolvableByMDP(const std::shared_ptr<MDPInterface> &mdp) : underlying_problem(mdp)
    {
        this->initial_state_  = std::static_pointer_cast<State>(*this->getUnderlyingProblem()->getStateSpace(0)->begin());
    }

    SolvableByMDP::~SolvableByMDP()
    {
    }

    void SolvableByMDP::setInitialState(const std::shared_ptr<State>& state)
    {
        this->initial_state_ = state;
    }


    std::shared_ptr<State> SolvableByMDP::getInitialState()
    {
        return this->initial_state_;
    }

    std::shared_ptr<State> SolvableByMDP::nextState(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t, const std::shared_ptr<HSVI> &hsvi) const
    {
        double max = -std::numeric_limits<double>::max();
        std::shared_ptr<State> argmax = 0;
        for (const auto &next_state : this->underlying_problem->getReachableStates(state, action, t))
        {
            double tmp = this->underlying_problem->getTransitionProbability(state, action, next_state, t) * hsvi->do_excess(next_state, 0, t + 1);
            if (tmp > max)
            {
                max = tmp;
                argmax = next_state;
            }
        }

        // for (const auto &pair_state_proba : state->expand(action))
        // {
        //     double tmp = pair_state_proba.second * hsvi->do_excess(pair_state_proba.first, 0, t + 1);
        //     if (tmp > max)
        //     {
        //         max = tmp;
        //         argmax = state_;
        //     }
        // }
        return argmax;
    }

    std::shared_ptr<Space> SolvableByMDP::getActionSpaceAt(const std::shared_ptr<State> &, number t)
    {
        return std::dynamic_pointer_cast<MDPInterface>(this->underlying_problem)->getActionSpace(t);
    }

    double SolvableByMDP::getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    {
        return this->underlying_problem->getReward(state, action, t);
    }

    double SolvableByMDP::getExpectedNextValue(const std::shared_ptr<ValueFunction> &value_function, const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    {
        double tmp = 0;
        double transition_proba =0;
        for (const auto &next_state : this->underlying_problem->getReachableStates(state, action, t))
        {
            transition_proba +=  this->underlying_problem->getTransitionProbability(state, action, next_state, t);
            tmp += this->underlying_problem->getTransitionProbability(state, action, next_state, t) * value_function->getValueAt(next_state, t + 1);
        }
        return tmp;
    }

    bool SolvableByMDP::isSerialized() const
    {
        return false;
    }

    const std::shared_ptr<MDPInterface> &SolvableByMDP::getUnderlyingProblem() const
    {
        return this->underlying_problem;
    }

    const std::shared_ptr<MDPInterface> &SolvableByMDP::getUnderlyingMDP() const
    {
        return this->underlying_problem;
    }

    double SolvableByMDP::getDiscount(number t)
    {
        return this->underlying_problem->getDiscount(t);
    }

    double SolvableByMDP::getWeightedDiscount(number t)
    {
        return std::pow(this->getDiscount(t), t);
    }

    double SolvableByMDP::do_excess(double, double lb, double ub, double, double error, number t)
    {
        return (ub - lb) - error / this->getWeightedDiscount(t);
    }

    std::shared_ptr<Action> SolvableByMDP::selectNextAction(const std::shared_ptr<ValueFunction> &, const std::shared_ptr<ValueFunction> &ub, const std::shared_ptr<State> &s, number h)
    {
        return ub->getBestAction(s, h);
    }

}
