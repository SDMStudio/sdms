#include <sdm/world/mdp.hpp>
#include <sdm/world/base/base_mdp.hpp>

namespace sdm
{

    MDP::MDP()
    {
    }

    MDP::MDP(const std::shared_ptr<MDPInterface> &mdp) : underlying_problem(mdp)
    {
    }

    MDP::~MDP()
    {
    }

    std::shared_ptr<State> MDP::getInitialState()
    {
        return nullptr;
    }

    std::shared_ptr<State> MDP::nextState(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t, const std::shared_ptr<HSVI> &hsvi) const
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

    std::shared_ptr<Space<std::shared_ptr<Action>>> MDP::getActionSpaceAt(const std::shared_ptr<State> &, number t)
    {
        return std::static_pointer_cast<BaseMDP>(this->underlying_problem)->getActionSpace(t);
    }

    double MDP::getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    {
        return this->underlying_problem->getReward(state, action, t);
    }

    double MDP::getExpectedNextValue(const std::shared_ptr<ValueFunction> &value_function, const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    {
        double tmp = 0;
        for (const auto &next_state : this->underlying_problem->getReachableStates(state, action, t))
        {
            tmp += this->underlying_problem->getTransitionProbability(state, action, next_state, t) * value_function->getValueAt(next_state, t + 1);
        }
        return tmp;
    }

    bool MDP::isSerialized() const
    {
        return false;
    }

    const std::shared_ptr<MDPInterface> &MDP::getUnderlyingProblem() const
    {
        return this->underlying_problem;
    }

    const std::shared_ptr<MDPInterface> &MDP::getUnderlyingMDP() const
    {
        return this->underlying_problem;
    }

    double MDP::getDiscount(number t)
    {
        return this->underlying_problem->getDiscount(t);
    }

    double MDP::getWeightedDiscount(number horizon)
    {
        return std::pow(this->getDiscount(horizon), horizon);
    }

    double MDP::do_excess(double, double lb, double ub, double, double error, number horizon)
    {
        return (ub - lb) - error / this->getWeightedDiscount(horizon);
    }

    std::shared_ptr<Action> MDP::selectNextAction(const std::shared_ptr<ValueFunction> &, const std::shared_ptr<ValueFunction> &ub, const std::shared_ptr<State> &s, number h)
    {
        return ub->getBestAction(s, h);
    }

}
