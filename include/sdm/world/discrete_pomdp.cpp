#include <sdm/world/discrete_pomdp.hpp>

namespace sdm
{

    DiscretePOMDP::DiscretePOMDP()
    {
    }

    DiscretePOMDP::DiscretePOMDP(std::shared_ptr<DiscreteSpace<number>> state_sp, std::shared_ptr<DiscreteSpace<number>> action_sp, std::shared_ptr<DiscreteSpace<number>> obs_sp, std::shared_ptr<StateDynamics> state_dyn, std::shared_ptr<ObservationDynamics> obs_dyn, std::shared_ptr<Reward> rew_f, std::discrete_distribution<number> start_distrib, number planning_horizon, double discount, Criterion criterion)
        : StochasticProcessBase<DiscreteSpace<number>, std::discrete_distribution<number>>(state_sp, start_distrib),
          PartiallyObservableDecisionProcess<DiscreteSpace<number>,
                                             DiscreteSpace<number>,
                                             DiscreteSpace<number>,
                                             StateDynamics,
                                             ObservationDynamics,
                                             Reward,
                                             std::discrete_distribution<number>>(state_sp,
                                                                                 action_sp,
                                                                                 obs_sp,
                                                                                 state_dyn,
                                                                                 obs_dyn,
                                                                                 rew_f,
                                                                                 start_distrib,
                                                                                 planning_horizon,
                                                                                 discount,
                                                                                 criterion)

    {
    }

    // // Other methods
    std::shared_ptr<DiscreteMDP> DiscretePOMDP::toMDP()
    {
        return std::make_shared<DiscreteMDP>(this->getStateSpace(), this->getActionSpace(), this->getStateDynamics(), this->getReward(), this->getStartDistrib(), this->getPlanningHorizon(), this->getDiscount(), this->getCriterion());
    }
    // std::shared_ptr<BeliefMDP> DiscretePOMDP::toBeliefMDP()
    // {
    // }

}