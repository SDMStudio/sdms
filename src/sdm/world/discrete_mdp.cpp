#include <sdm/world/discrete_mdp.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/exception.hpp>

namespace sdm
{

    DiscreteMDP::DiscreteMDP()
    {
    }

    DiscreteMDP::DiscreteMDP(std::shared_ptr<DiscreteSpace<number>> state_sp, std::shared_ptr<DiscreteSpace<number>> action_sp)
        : StochasticProcessBase<DiscreteSpace<number>, std::discrete_distribution<number>>(state_sp),
          FullyObservableDecisionProcess<DiscreteSpace<number>, DiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>(state_sp, action_sp)
    {
        this->reset();
    }

    DiscreteMDP::DiscreteMDP(std::shared_ptr<DiscreteSpace<number>> state_sp, std::shared_ptr<DiscreteSpace<number>> action_sp, std::discrete_distribution<number> start_distrib)
        : StochasticProcessBase<DiscreteSpace<number>, std::discrete_distribution<number>>(state_sp, start_distrib),
          FullyObservableDecisionProcess<DiscreteSpace<number>, DiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>(state_sp, action_sp, start_distrib)
    {
        this->reset();
    }

    DiscreteMDP::DiscreteMDP(std::shared_ptr<DiscreteSpace<number>> state_sp, std::shared_ptr<DiscreteSpace<number>> action_sp, std::shared_ptr<StateDynamics> state_dyn, std::shared_ptr<Reward> rew, std::discrete_distribution<number> start_distrib, number planning_horizon, double discount, Criterion criterion)
        : StochasticProcessBase<DiscreteSpace<number>, std::discrete_distribution<number>>(state_sp, start_distrib),
          FullyObservableDecisionProcess<DiscreteSpace<number>, DiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>(state_sp, action_sp, state_dyn, rew, start_distrib, planning_horizon, discount, criterion)
    {
        this->reset();
    }

    DiscreteMDP::DiscreteMDP(std::string &filename)
    {
        *this = *(parser::parse_file(filename)->toPOMDP()->toMDP());
        this->reset();
    }

    // SolvableByHSVI interface implementation
    number DiscreteMDP::getInitialState()
    {
        return this->getInternalState();
    }

    number DiscreteMDP::nextState(const number &state, const number &action, number t, std::shared_ptr<HSVI<number, number>> hsvi) const
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

    std::shared_ptr<DiscreteSpace<number>> DiscreteMDP::getActionSpaceAt(const number &)
    {
        return this->getActionSpace();
    }

    std::shared_ptr<Reward> DiscreteMDP::getReward() const
    {
        return FullyObservableDecisionProcess<DiscreteSpace<number>, DiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>::getReward();
    }
    double DiscreteMDP::getReward(const number &state, const number &action) const
    {
        return FullyObservableDecisionProcess<DiscreteSpace<number>, DiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>::getReward()->getReward(state, action);
    }

    double DiscreteMDP::getExpectedNextValue(std::shared_ptr<ValueFunction<number, number>> value_function, const number &state, const number &action, number t) const
    {
        double tmp = 0;
        for (number state_ = 0; state_ < this->getStateSpace()->getNumItems(); state_++)
        {
            tmp += this->getStateDynamics()->getTransitionProbability(state, action, state_) * value_function->getValueAt(state_, t + 1);
        }
        return tmp;
    }

    DiscreteMDP *DiscreteMDP::getUnderlyingProblem()
    {
        return this;
    }

    bool DiscreteMDP::isSerialized() const
    {
        return false;
    }

    std::shared_ptr<DiscreteMDP> DiscreteMDP::getptr()
    {
        return shared_from_this();
    }

    std::shared_ptr<DiscreteMDP> DiscreteMDP::toMDP()
    {
        return this->getptr();
    }

    std::shared_ptr<BeliefMDP<BeliefState<number>, number, number>> DiscreteMDP::toBeliefMDP()
    {
        throw sdm::exception::NotImplementedException();
    }

    double DiscreteMDP::getDiscount(number)
    {
        return this->discount_;
    }

    double DiscreteMDP::getWeightedDiscount(number horizon)
    {
        return std::pow(this->getDiscount(), horizon);
    }

    double DiscreteMDP::do_excess(double, double lb, double ub, double, double error, number horizon)
    {
        return (ub - lb) - error / this->getWeightedDiscount(horizon);
    }

    number DiscreteMDP::selectNextAction(const std::shared_ptr<ValueFunction<number, number>> &, const std::shared_ptr<ValueFunction<number, number>> &ub, const number &s, number h)
    {
        return ub->getBestAction(s, h);
    }
}