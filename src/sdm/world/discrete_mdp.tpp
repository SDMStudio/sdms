#include <sdm/world/discrete_mdp.hpp>

namespace sdm
{

    template <typename TState, typename TAction>
    MDP<TState, TAction>::MDP()
    {
    }

    template <typename TState, typename TAction>
    MDP<TState, TAction>::MDP(std::shared_ptr<DiscreteSpace<TState>> state_sp, std::shared_ptr<DiscreteSpace<TAction>> action_sp)
        : StochasticProcessBase<DiscreteSpace<TState>, std::discrete_distribution<number>>(state_sp),
          FullyObservableDecisionProcess<DiscreteSpace<TState>, DiscreteSpace<TAction>, StateDynamics, Reward, std::discrete_distribution<number>>(state_sp, action_sp)
    {
        this->reset();
    }

    template <typename TState, typename TAction>
    MDP<TState, TAction>::MDP(std::shared_ptr<DiscreteSpace<TState>> state_sp, std::shared_ptr<DiscreteSpace<TAction>> action_sp, std::discrete_distribution<number> start_distrib)
        : StochasticProcessBase<DiscreteSpace<TState>, std::discrete_distribution<number>>(state_sp, start_distrib),
          FullyObservableDecisionProcess<DiscreteSpace<TState>, DiscreteSpace<TAction>, StateDynamics, Reward, std::discrete_distribution<number>>(state_sp, action_sp, start_distrib)
    {
        this->reset();
    }

    template <typename TState, typename TAction>
    MDP<TState, TAction>::MDP(std::shared_ptr<DiscreteSpace<TState>> state_sp, std::shared_ptr<DiscreteSpace<TAction>> action_sp, std::shared_ptr<StateDynamics> state_dyn, std::shared_ptr<Reward> rew, std::discrete_distribution<number> start_distrib, number planning_horizon, double discount, Criterion criterion)
        : StochasticProcessBase<DiscreteSpace<TState>, std::discrete_distribution<number>>(state_sp, start_distrib),
          FullyObservableDecisionProcess<DiscreteSpace<TState>, DiscreteSpace<TAction>, StateDynamics, Reward, std::discrete_distribution<number>>(state_sp, action_sp, state_dyn, rew, start_distrib, planning_horizon, discount, criterion)
    {
        this->reset();
    }

    template <typename TState, typename TAction>
    MDP<TState, TAction>::MDP(std::string &filename)
    {
        *this = *(DiscreteDecPOMDP(filename).toPOMDP()->toMDP());
        this->reset();
    }

    // SolvableByHSVI interface implementation
    template <typename TState, typename TAction>
    TState MDP<TState, TAction>::getInitialState()
    {
        return this->getInternalState();
    }

    template <typename TState, typename TAction>
    TState MDP<TState, TAction>::nextState(const TState &state, const TAction &action, number t, std::shared_ptr<HSVI<TState, TAction>> hsvi) const
    {
        double max = -std::numeric_limits<double>::max();
        number amax = 0;
        for (number state_ = 0; state_ < this->getStateSpace()->getNumItems(); state_++)
        {
            double tmp = this->getStateDynamics()->getTransitionProbability(state, action, state_) * hsvi->do_excess(state_, 0, t + 1);
            if (tmp > max)
            {
                max = tmp;
                amax = state_;
            }
        }

        // for (const auto &pair_state_proba : state->expand(action))
        // {
        //     double tmp = pair_state_proba.second * hsvi->do_excess(pair_state_proba.first, 0, t + 1);
        //     if (tmp > max)
        //     {
        //         max = tmp;
        //         amax = state_;
        //     }
        // }
        return amax;
    }

    template <typename TState, typename TAction>
    std::shared_ptr<DiscreteSpace<TAction>> MDP<TState, TAction>::getActionSpaceAt(const TState &)
    {
        return this->getActionSpace();
    }

    template <typename TState, typename TAction>
    std::shared_ptr<Reward> MDP<TState, TAction>::getReward() const
    {
        return FullyObservableDecisionProcess<DiscreteSpace<number>, DiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>::getReward();
    }
    template <typename TState, typename TAction>
    double MDP<TState, TAction>::getReward(const TState &state, const TAction &action) const
    {
        return FullyObservableDecisionProcess<DiscreteSpace<number>, DiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>::getReward()->getReward(state, action);
    }

    template <typename TState, typename TAction>
    double MDP<TState, TAction>::getExpectedNextValue(std::shared_ptr<ValueFunction<TState, TAction>> value_function, const TState &state, const TAction &action, number t) const
    {
        double tmp = 0;
        for (number state_ = 0; state_ < this->getStateSpace()->getNumItems(); state_++)
        {
            tmp += this->getStateDynamics()->getTransitionProbability(state, action, state_) * value_function->getValueAt(state_, t + 1);
        }
        return tmp;
    }

    template <typename TState, typename TAction>
    bool MDP<TState, TAction>::isSerialized() const
    {
        return false;
    }

    template <typename TState, typename TAction>
    MDP<number, number> * MDP<TState, TAction>::getUnderlyingProblem()
    {
        return nullptr;
        // return this->getptr().get();
    }

    template <typename TState, typename TAction>
    std::shared_ptr<MDP<TState, TAction>> MDP<TState, TAction>::getptr()
    {
        return this->shared_from_this();
    }

    template <typename TState, typename TAction>
    std::shared_ptr<MDP<TState, TAction>> MDP<TState, TAction>::toMDP()
    {
        return this->getptr();
    }

    template <typename TState, typename TAction>
    double MDP<TState, TAction>::getDiscount(number)
    {
        return this->discount_;
    }

    template <typename TState, typename TAction>
    double MDP<TState, TAction>::getWeightedDiscount(number horizon)
    {
        return std::pow(this->getDiscount(), horizon);
    }

    template <typename TState, typename TAction>
    double MDP<TState, TAction>::do_excess(double, double lb, double ub, double, double error, number horizon)
    {
        return (ub - lb) - error / this->getWeightedDiscount(horizon);
    }

    template <typename TState, typename TAction>
    TAction MDP<TState, TAction>::selectNextAction(const std::shared_ptr<ValueFunction<TState, TAction>> &, const std::shared_ptr<ValueFunction<TState, TAction>> &ub, const TState &s, number h)
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