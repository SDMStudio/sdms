#include <sdm/world/mpomdp.hpp>
#include <sdm/core/joint.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>

namespace sdm
{

    MPOMDP::MPOMDP(double horizon,
                   double discount,
                   const std::shared_ptr<Space> &state_space,
                   const std::shared_ptr<Space> &action_space,
                   const std::shared_ptr<Space> &obs_space,
                   const std::shared_ptr<RewardInterface> &reward,
                   const std::shared_ptr<StateDynamicsInterface> &state_dynamics,
                   const std::shared_ptr<ObservationDynamicsInterface> &obs_dynamics,
                   const std::shared_ptr<Distribution<std::shared_ptr<State>>> &start_distrib)
        : MDP(horizon, discount, state_space, action_space, reward, state_dynamics, start_distrib),
          POMDP(horizon, discount, state_space, action_space, obs_space, reward, state_dynamics, obs_dynamics, start_distrib),
          MMDP(horizon, discount, state_space, action_space, reward, state_dynamics, start_distrib)
    {
        this->num_agents_ = std::static_pointer_cast<MultiDiscreteSpace>(action_space)->getNumSpaces();
    }

    std::shared_ptr<Space> MPOMDP::getObservationSpace(number t) const
    {
        return POMDP::getObservationSpace(t);
    }
    std::shared_ptr<Space> MPOMDP::getObservationSpace(number agent_id, number t) const
    {
        return std::static_pointer_cast<MultiDiscreteSpace>(this->getObservationSpace(t))->getSpace(agent_id);
    }

}