#include <sdm/world/discrete_decpomdp.hpp>

namespace sdm
{

    DiscreteDecPOMDP::DiscreteDecPOMDP()
    {
    }

    DiscreteDecPOMDP::DiscreteDecPOMDP(std::shared_ptr<DiscreteSpace<number>> state_sp, std::shared_ptr<MultiDiscreteSpace<number>> action_sp, std::shared_ptr<MultiDiscreteSpace<number>> obs_sp, std::shared_ptr<StateDynamics> state_dyn, std::shared_ptr<ObservationDynamics> obs_dyn, std::shared_ptr<Reward> rew_f, std::discrete_distribution<number> start_distrib, number planning_horizon, double discount, Criterion criterion)
        : StochasticProcessBase<DiscreteSpace<number>, std::discrete_distribution<number>>(state_sp, start_distrib),
          PartiallyObservableDecisionProcess<DiscreteSpace<number>,
                                             MultiDiscreteSpace<number>,
                                             MultiDiscreteSpace<number>,
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
    std::shared_ptr<DiscretePOMDP> DiscreteDecPOMDP::toPOMDP()
    {
        std::shared_ptr<DiscreteSpace<number>> new_action_space = std::make_shared<DiscreteSpace<number>>(this->getActionSpace()->getNumJointItems());
        std::shared_ptr<DiscreteSpace<number>> new_obs_space = std::make_shared<DiscreteSpace<number>>(this->getObsSpace()->getNumJointItems());
        return std::make_shared<DiscretePOMDP>(this->getStateSpace(), new_action_space, new_obs_space, this->getStateDynamics(), this->getObsDynamics(), this->getReward(), this->getStartDistrib(), this->getPlanningHorizon(), this->getDiscount(), this->getCriterion());
    }

    std::shared_ptr<DiscreteMMDP> DiscreteDecPOMDP::toMMDP()
    {
        return std::make_shared<DiscreteMMDP>(this->getStateSpace(), this->getActionSpace(), this->getStateDynamics(), this->getReward(), this->getStartDistrib(), this->getPlanningHorizon(), this->getDiscount(), this->getCriterion());
    }

    // std::shared_ptr<BeliefMDP> DiscreteDecPOMDP::toBeliefMDP()
    // {
    // }

}