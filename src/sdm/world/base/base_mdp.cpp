#include <sdm/world/base/base_mdp.hpp>

namespace sdm
{

    BaseMDP::BaseMDP(number num_agents,
                     double discount,
                     const std::shared_ptr<Space<std::shared_ptr<State>>> &state_space,
                     const std::shared_ptr<Space<std::shared_ptr<Action>>> &action_space,
                     const std::shared_ptr<BaseReward> &reward,
                     const std::shared_ptr<BaseStateDynamics> &state_dynamics,
                     const std::shared_ptr<Distribution<std::shared_ptr<State>>> &start_distrib) : num_agents_(num_agents),
                                                                                                   discount_(discount),
                                                                                                   state_space_(state_space),
                                                                                                   action_space_(action_space),
                                                                                                   reward_(reward),
                                                                                                   state_dynamics_(state_dynamics),
                                                                                                   start_distrib_(start_distrib)

    {
    }

    BaseMDP::~BaseMDP() {}

    number BaseMDP::getNumAgents() const
    {
        return this->num_agents_;
    }

    double BaseMDP::getDiscount(number t) const
    {
        return this->discount_;
    }

    std::shared_ptr<Distribution<std::shared_ptr<State>>> BaseMDP::getStartDistribution() const
    {
        return this->start_distrib_;
    }

    std::vector<std::shared_ptr<State>> BaseMDP::getAllStates(number t) const
    {
        return this->getStateSpace(t)->getAll();
    }

    std::vector<std::shared_ptr<Action>> BaseMDP::getAllActions(number t) const
    {
        return this->getActionSpace(t)->getAll();
    }

    std::set<std::shared_ptr<State>> BaseMDP::getReachableStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    {
        return this->state_dynamics_->getReachableStates(state, action, t);
    }

    double BaseMDP::getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    {
        return this->reward_->getReward(state, action, t);
    }

    double BaseMDP::getTransitionProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t) const
    {
        return this->state_dynamics_->getTransitionProbability(state, action, next_state, t);
    }

    const std::shared_ptr<Space<std::shared_ptr<Action>>> &BaseMDP::getActionSpace(number) const
    {
        return this->action_space_;
    }

    const std::shared_ptr<Space<std::shared_ptr<State>>> &BaseMDP::getStateSpace(number) const
    {
        return this->state_space_;
    }
}