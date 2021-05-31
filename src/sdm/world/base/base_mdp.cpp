#include <sdm/world/base/base_mdp.hpp>

namespace sdm
{

    MDP::MDP()
    {
    }

    MDP::MDP(std::shared_ptr<DiscreteSpace<TState>> state_sp, std::shared_ptr<DiscreteSpace<TAction>> action_sp)
        : StochasticProcessBase<DiscreteSpace<TState>, std::discrete_distribution<number>>(state_sp),
          FullyObservableDecisionProcess<DiscreteSpace<TState>, DiscreteSpace<TAction>, StateDynamics, Reward, std::discrete_distribution<number>>(state_sp, action_sp)
    {
        this->reset();
    }

    MDP::MDP(std::shared_ptr<DiscreteSpace<TState>> state_sp, std::shared_ptr<DiscreteSpace<TAction>> action_sp, std::discrete_distribution<number> start_distrib)
        : StochasticProcessBase<DiscreteSpace<TState>, std::discrete_distribution<number>>(state_sp, start_distrib),
          FullyObservableDecisionProcess<DiscreteSpace<TState>, DiscreteSpace<TAction>, StateDynamics, Reward, std::discrete_distribution<number>>(state_sp, action_sp, start_distrib)
    {
        this->reset();
    }

    MDP::MDP(std::shared_ptr<DiscreteSpace<TState>> state_sp, std::shared_ptr<DiscreteSpace<TAction>> action_sp, std::shared_ptr<StateDynamics> state_dyn, std::shared_ptr<Reward> rew, std::discrete_distribution<number> start_distrib, number planning_horizon, double discount, Criterion criterion)
        : StochasticProcessBase<DiscreteSpace<TState>, std::discrete_distribution<number>>(state_sp, start_distrib),
          FullyObservableDecisionProcess<DiscreteSpace<TState>, DiscreteSpace<TAction>, StateDynamics, Reward, std::discrete_distribution<number>>(state_sp, action_sp, state_dyn, rew, start_distrib, planning_horizon, discount, criterion)
    {
        this->reset();
    }

    MDP::MDP(std::string &filename)
    {
        *this = *(DiscreteDecPOMDP(filename).toMDP());
        this->reset();
    }

    // SolvableByHSVI interface implementation

    std::shared_ptr<State> MDP::getInitialState()
    {
        return this->getInternalState();
    }

    std::shared_ptr<State> MDP::nextState(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t, std::shared_ptr<HSVI> hsvi) const
    {
        double max = -std::numeric_limits<double>::max();
        std::shared_ptr<State> argmax = 0;
        for (const auto &next_state : this->getReachableStates(state, action, t))
        {
            double tmp = this->getTransitionProbability(state, action, next_state, t) * hsvi->do_excess(next_state, 0, t + 1);
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

    std::shared_ptr<DiscreteSpace<TAction>> MDP::getActionSpaceAt(const TState &)
    {
        return this->getActionSpace();
    }

    std::shared_ptr<Reward> MDP::getReward() const
    {
        return FullyObservableDecisionProcess<DiscreteSpace<number>, DiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>::getReward();
    }

    double MDP::getReward(const TState &state, const TAction &action) const
    {
        return FullyObservableDecisionProcess<DiscreteSpace<number>, DiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>::getReward()->getReward(state, action);
    }

    double MDP::getExpectedNextValue(std::shared_ptr<ValueFunction<TState, TAction>> value_function, const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    {
        double tmp = 0;
        for (const auto &next_state : this->getReachableStates(state, action, t))
        {
            tmp += this->getTransitionProbability(state, action, next_state, t) * value_function->getValueAt(next_state, t + 1);
        }
        return tmp;
    }

    bool MDP::isSerialized() const
    {
        return false;
    }

    std::shared_ptr<MDP> MDP::getUnderlyingProblem()
    {
        return this->getptr();
    }

    std::shared_ptr<MDP> MDP::getptr()
    {
        return this->shared_from_this();
    }

    std::shared_ptr<MDP> MDP::toMDP()
    {
        return this->getptr();
    }

    double MDP::getDiscount(number)
    {
        return this->discount_;
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
// #include <sdm/world/discrete_mdp.hpp>
// #include <sdm/parser/parser.hpp>
// #include <sdm/exception.hpp>

// namespace sdm
// {

//     DiscreteMDP::DiscreteMDP()
//     {
//     }

//     DiscreteMDP::DiscreteMDP(std::shared_ptr<DiscreteSpace<number>> state_sp, std::shared_ptr<DiscreteSpace<number>> action_sp)
//         : StochasticProcessBase<DiscreteSpace<number>, std::discrete_distribution<number>>(state_sp),
//           FullyObservableDecisionProcess<DiscreteSpace<number>, DiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>(state_sp, action_sp)
//     {
//         this->reset();
//     }

//     DiscreteMDP::DiscreteMDP(std::shared_ptr<DiscreteSpace<number>> state_sp, std::shared_ptr<DiscreteSpace<number>> action_sp, std::discrete_distribution<number> start_distrib)
//         : StochasticProcessBase<DiscreteSpace<number>, std::discrete_distribution<number>>(state_sp, start_distrib),
//           FullyObservableDecisionProcess<DiscreteSpace<number>, DiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>(state_sp, action_sp, start_distrib)
//     {
//         this->reset();
//     }

//     DiscreteMDP::DiscreteMDP(std::shared_ptr<DiscreteSpace<number>> state_sp, std::shared_ptr<DiscreteSpace<number>> action_sp, std::shared_ptr<StateDynamics> state_dyn, std::shared_ptr<Reward> rew, std::discrete_distribution<number> start_distrib, number planning_horizon, double discount, Criterion criterion)
//         : StochasticProcessBase<DiscreteSpace<number>, std::discrete_distribution<number>>(state_sp, start_distrib),
//           FullyObservableDecisionProcess<DiscreteSpace<number>, DiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>(state_sp, action_sp, state_dyn, rew, start_distrib, planning_horizon, discount, criterion)
//     {
//         this->reset();
//     }

//     DiscreteMDP::DiscreteMDP(std::string &filename)
//     {
//         *this = *(parser::parse_file(filename)->toPOMDP()->toMDP());
//         this->reset();
//     }

//     // SolvableByHSVI interface implementation
//     number DiscreteMDP::getInitialState()
//     {
//         return this->getInternalState();
//     }

//     number DiscreteMDP::nextState(const number &state, const number &action, number t, std::shared_ptr<HSVI<number, number>> hsvi) const
//     {
//         double max = -std::numeric_limits<double>::max();
//         number amax = 0;
//         for (number state_ = 0; state_ < this->getStateSpace()->getNumItems(); state_++)
//         {
//             double tmp = this->getStateDynamics()->getTransitionProbability(state, action, state_) * hsvi->do_excess(state_, 0, t + 1);
//             if (tmp > max)
//             {
//                 max = tmp;
//                 amax = state_;
//             }
//         }

//         // for (const auto &pair_state_proba : state->expand(action))
//         // {
//         //     double tmp = pair_state_proba.second * hsvi->do_excess(pair_state_proba.first, 0, t + 1);
//         //     if (tmp > max)
//         //     {
//         //         max = tmp;
//         //         amax = state_;
//         //     }
//         // }
//         return amax;
//     }

//     std::shared_ptr<DiscreteSpace<number>> DiscreteMDP::getActionSpaceAt(const number &)
//     {
//         return this->getActionSpace();
//     }

//     std::shared_ptr<Reward> DiscreteMDP::getReward() const
//     {
//         return FullyObservableDecisionProcess<DiscreteSpace<number>, DiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>::getReward();
//     }
//     double DiscreteMDP::getReward(const number &state, const number &action) const
//     {
//         return FullyObservableDecisionProcess<DiscreteSpace<number>, DiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>::getReward()->getReward(state, action);
//     }

//     double DiscreteMDP::getExpectedNextValue(std::shared_ptr<ValueFunction<number, number>> value_function, const number &state, const number &action, number t) const
//     {
//         double tmp = 0;
//         for (number state_ = 0; state_ < this->getStateSpace()->getNumItems(); state_++)
//         {
//             tmp += this->getStateDynamics()->getTransitionProbability(state, action, state_) * value_function->getValueAt(state_, t + 1);
//         }
//         return tmp;
//     }

//     DiscreteMDP *DiscreteMDP::getUnderlyingProblem()
//     {
//         return this;
//     }

//     bool DiscreteMDP::isSerialized() const
//     {
//         return false;
//     }

//     std::shared_ptr<DiscreteMDP> DiscreteMDP::getptr()
//     {
//         return shared_from_this();
//     }

//     std::shared_ptr<DiscreteMDP> DiscreteMDP::toMDP()
//     {
//         return this->getptr();
//     }

//     std::shared_ptr<BeliefMDP<BeliefState<number>, number, number>> DiscreteMDP::toBeliefMDP()
//     {
//         throw sdm::exception::NotImplementedException();
//     }

//     double DiscreteMDP::getDiscount(number)
//     {
//         return this->discount_;
//     }

//     double DiscreteMDP::getWeightedDiscount(number horizon)
//     {
//         return std::pow(this->getDiscount(), horizon);
//     }

//     double DiscreteMDP::do_excess(double, double lb, double ub, double, double error, number horizon)
//     {
//         return (ub - lb) - error / this->getWeightedDiscount(horizon);
//     }

//     number DiscreteMDP::selectNextAction(const std::shared_ptr<ValueFunction<number, number>> &, const std::shared_ptr<ValueFunction<number, number>> &ub, const number &s, number h)
//     {
//         return ub->getBestAction(s, h);
//     }
// }