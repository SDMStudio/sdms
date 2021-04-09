#include <sdm/world/discrete_mmdp.hpp>
#include <sdm/parser/parser.hpp>

namespace sdm
{

    DiscreteMMDP::DiscreteMMDP()
    {
    }

    DiscreteMMDP::DiscreteMMDP(std::shared_ptr<DiscreteSpace<number>> state_sp, std::shared_ptr<MultiDiscreteSpace<number>> action_sp)
        : StochasticProcessBase<DiscreteSpace<number>, std::discrete_distribution<number>>(state_sp),
          FullyObservableDecisionProcess<DiscreteSpace<number>, MultiDiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>(state_sp, action_sp)
    {
    }

    DiscreteMMDP::DiscreteMMDP(std::shared_ptr<DiscreteSpace<number>> state_sp, std::shared_ptr<MultiDiscreteSpace<number>> action_sp, std::discrete_distribution<number> start_distrib)
        : StochasticProcessBase<DiscreteSpace<number>, std::discrete_distribution<number>>(state_sp, start_distrib),
          FullyObservableDecisionProcess<DiscreteSpace<number>, MultiDiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>(state_sp, action_sp, start_distrib)
    {
    }

    DiscreteMMDP::DiscreteMMDP(std::shared_ptr<DiscreteSpace<number>> state_sp, std::shared_ptr<MultiDiscreteSpace<number>> action_sp, std::shared_ptr<StateDynamics> state_dyn, std::shared_ptr<Reward> rew, std::discrete_distribution<number> start_distrib, number planning_horizon, double discount, Criterion criterion)
        : StochasticProcessBase<DiscreteSpace<number>, std::discrete_distribution<number>>(state_sp, start_distrib),
          FullyObservableDecisionProcess<DiscreteSpace<number>, MultiDiscreteSpace<number>, StateDynamics, Reward, std::discrete_distribution<number>>(state_sp, action_sp, state_dyn, rew, start_distrib, planning_horizon, discount, criterion)
    {
    }

    DiscreteMMDP::DiscreteMMDP(std::string &filename)
    {
        *this = *(parser::parse_file(filename.c_str())->toMMDP());
    }

    std::shared_ptr<DiscreteMDP> DiscreteMMDP::toMDP()
    {
        std::shared_ptr<DiscreteSpace<number>> new_action_space = std::make_shared<DiscreteSpace<number>>(this->getActionSpace()->getNumJointItems());
        return std::make_shared<DiscreteMDP>(this->getStateSpace(), new_action_space, this->getStateDynamics(), this->getReward(), this->getStartDistrib(), this->getPlanningHorizon(), this->getDiscount(), this->getCriterion());
    }

    std::shared_ptr<BeliefMDP<BeliefState, number, number>> DiscreteMMDP::toBeliefMDP()
    {
        throw sdm::exception::NotImplementedException();
    }

}