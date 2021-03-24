#include <sdm/world/discrete_mdp.hpp>
#include <sdm/parser/parser.hpp>

namespace sdm
{

    DiscreteMDP::DiscreteMDP()
    {
    }

    DiscreteMDP::DiscreteMDP(std::shared_ptr<DiscreteSpace<number>> state_sp, std::shared_ptr<DiscreteSpace<number>> action_sp)
        : StochasticProcessBase<DiscreteSpace<number>, std::discrete_distribution<number>>(state_sp),
          FullyObservableDecisionProcess<DiscreteSpace<number>, DiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>(state_sp, action_sp)
    {
    }

    DiscreteMDP::DiscreteMDP(std::shared_ptr<DiscreteSpace<number>> state_sp, std::shared_ptr<DiscreteSpace<number>> action_sp, std::discrete_distribution<number> start_distrib)
        : StochasticProcessBase<DiscreteSpace<number>, std::discrete_distribution<number>>(state_sp, start_distrib),
          FullyObservableDecisionProcess<DiscreteSpace<number>, DiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>(state_sp, action_sp, start_distrib)
    {
    }

    DiscreteMDP::DiscreteMDP(std::shared_ptr<DiscreteSpace<number>> state_sp, std::shared_ptr<DiscreteSpace<number>> action_sp, std::shared_ptr<StateDynamics> state_dyn, std::shared_ptr<Reward> rew, std::discrete_distribution<number> start_distrib, number planning_horizon, double discount, Criterion criterion)
        : StochasticProcessBase<DiscreteSpace<number>, std::discrete_distribution<number>>(state_sp, start_distrib),
          FullyObservableDecisionProcess<DiscreteSpace<number>, DiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>(state_sp, action_sp, state_dyn, rew, start_distrib, planning_horizon, discount, criterion)
    {
    }

    DiscreteMDP::DiscreteMDP(std::string &filename)
    {
        *this = *(parser::parse_file(filename.c_str())->toPOMDP()->toMDP());
    }

    // SolvableByHSVI interface implementation
    number DiscreteMDP::getInitialState()
    {
        return this->getInternalState();
    }

    number DiscreteMDP::nextState(const number &state, const number &action, int t, HSVI<number, number> *hsvi) const
    {
        double max = std::numeric_limits<double>::min();
        number amax = 0;
        for (number state_ = 0; state_ < this->getStateSpace()->getNumItems(); state_++)
        {
            double tmp = this->getStateDynamics()->getTransitionProbability(state, action, state_) * hsvi->do_excess(state_, t + 1);
            if (tmp > max)
            {
                max = tmp;
                amax = state_;
            }
        }
        return amax;
    }

    std::shared_ptr<DiscreteSpace<number>> DiscreteMDP::getActionSpaceAt(const number &state)
    {
        return this->getActionSpace();
    }

    std::shared_ptr<Reward> DiscreteMDP::getReward() const
    {
        return FullyObservableDecisionProcess<DiscreteSpace<number>, DiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>::getReward();
    }

    double DiscreteMDP::getReward(const number &state, const number &action) const
    {
        return this->getReward()->getReward(state, action);
    }

    double DiscreteMDP::getExpectedNextValue(ValueFunction<number, number> *value_function, const number &state, const number &action, int t) const
    {
        double tmp = 0;
        for (number state_ = 0; state_ < this->getStateSpace()->getNumItems(); state_++)
        {
            tmp += this->getStateDynamics()->getTransitionProbability(state, action, state_) * value_function->getValueAt(state_, t + 1);
        }
        return tmp;
    }

    double DiscreteMDP::getDiscount()
    {
        return FullyObservableDecisionProcess<DiscreteSpace<number>, DiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>::getDiscount();
    }

    void DiscreteMDP::setDiscount(double discount)
    {
        FullyObservableDecisionProcess<DiscreteSpace<number>, DiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>::setDiscount(discount);
    }

    DiscreteMDP *DiscreteMDP::getUnderlyingProblem()
    {
        return this;
    }
    
}