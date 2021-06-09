#include <sdm/world/mmdp.hpp>
#include <sdm/core/joint.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>

namespace sdm
{

    MMDP::MMDP(const std::shared_ptr<Space> &state_space,
               const std::shared_ptr<Space> &action_space,
               const std::shared_ptr<RewardInterface> &reward,
               const std::shared_ptr<StateDynamicsInterface> &state_dynamics,
               const std::shared_ptr<Distribution<std::shared_ptr<State>>> &start_distrib,
               number horizon,
               double discount,
               Criterion criterion)
        : MDP(state_space, action_space, reward, state_dynamics, start_distrib, horizon, discount, criterion)
    {
        this->num_agents_ = std::static_pointer_cast<MultiDiscreteSpace>(action_space)->getNumSpaces();
    }

    std::shared_ptr<Space> MMDP::getActionSpace(number t) const
    {
        return MDP::getActionSpace(t);
    }
    std::shared_ptr<Space> MMDP::getActionSpace(number agent_id, number t) const
    {
        return std::static_pointer_cast<MultiDiscreteSpace>(this->getActionSpace(t))->getSpace(agent_id);
    }

}