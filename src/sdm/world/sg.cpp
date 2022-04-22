#include <sdm/types.hpp>
#include <sdm/core/space/space.hpp>
#include <sdm/world/sg.hpp>

namespace sdm
{
    SG::SG() {}

    SG::SG(const std::shared_ptr<StateSpace> &state_space,
           const std::shared_ptr<ActionSpace> &action_space,
           const std::shared_ptr<RewardModel> &reward,
           const std::shared_ptr<StateDynamicsInterface> &state_dynamics,
           const std::shared_ptr<Distribution<std::shared_ptr<State>>> &start_distrib,
           number horizon,
           double discount,
           Criterion criterion) : MMDP(state_space, action_space, reward, state_dynamics, start_distrib, horizon, discount, criterion) {}

    double SG::getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    {
        return MMDP::getReward(state, action, t);
    }

    double SG::getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number agent_id, number t) const
    {
        return this->reward_space_->getReward(state, action, agent_id, t);
    }
} // namespace sdm
