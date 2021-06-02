
namespace sdm
{

    MDP::MDP()
    {
    }

    MDP(const std::shared_ptr<MDPInterface> &mdp) : mdp_(mdp)
    {
    }

    std::shared_ptr<State> MDP::nextState(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t, std::shared_ptr<HSVI> hsvi) const
    {
        double max = -std::numeric_limits<double>::max();
        std::shared_ptr<State> argmax = 0;
        for (const auto &next_state : this->getUnderlyingProblem()->getReachableStates(state, action, t))
        {
            double tmp = this->getUnderlyingProblem()->getTransitionProbability(state, action, next_state, t) * hsvi->do_excess(next_state, 0, t + 1);
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

    std::shared_ptr<DiscreteSpace<TAction>> MDP::getActionSpaceAt(const TState &, number t)
    {
        return this->getUnderlyingProblem()->getActionSpace(t);
    }

    double MDP::getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    {
        return this->getUnderlyingProblem()->getReward(state, action, t);
    }

    double MDP::getExpectedNextValue(std::shared_ptr<ValueFunction<TState, TAction>> value_function, const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    {
        double tmp = 0;
        for (const auto &next_state : this->getUnderlyingProblem()->getReachableStates(state, action, t))
        {
            tmp += this->getUnderlyingProblem()->getTransitionProbability(state, action, next_state, t) * value_function->getValueAt(next_state, t + 1);
        }
        return tmp;
    }

    bool MDP::isSerialized() const
    {
        return false;
    }

    std::shared_ptr<MDPInterface> MDP::getUnderlyingProblem()
    {
        return this->mdp_;
    }

    std::shared_ptr<MDPInterface> MDP::getUnderlyingMDP()
    {
        return this->getUnderlyingProblem();
    }

    double MDP::getDiscount(number t)
    {
        return this->getUnderlyingProblem()->getDiscount(t);
    }

    double MDP::getWeightedDiscount(number horizon)
    {
        return std::pow(this->getDiscount(horizon), horizon);
    }

    double MDP::do_excess(double, double lb, double ub, double, double error, number horizon)
    {
        return (ub - lb) - error / this->getWeightedDiscount(horizon);
    }

    TAction MDP::selectNextAction(const std::shared_ptr<ValueFunction<TState, TAction>> &, const std::shared_ptr<ValueFunction<TState, TAction>> &ub, const TState &s, number h)
    {
        return ub->getBestAction(s, h);
    }


}
