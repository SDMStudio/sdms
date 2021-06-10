#include <sdm/world/mdp.hpp>

namespace sdm
{

    MDP::MDP(const std::shared_ptr<Space> &state_space,
             const std::shared_ptr<Space> &action_space,
             const std::shared_ptr<RewardInterface> &reward,
             const std::shared_ptr<StateDynamicsInterface> &state_dynamics,
             const std::shared_ptr<Distribution<std::shared_ptr<State>>> &start_distrib,
             number horizon,
             double discount,
             Criterion criterion) : num_agents_(1),
                                    horizon_(horizon),
                                    discount_(discount),
                                    criterion_(criterion),
                                    state_space_(state_space),
                                    action_space_(action_space),
                                    reward_(reward),
                                    state_dynamics_(state_dynamics),
                                    start_distrib_(start_distrib)

    {
    }

    MDP::~MDP() {}

    number MDP::getNumAgents() const
    {
        return this->num_agents_;
    }

    double MDP::getDiscount(number) const
    {
        return this->discount_;
    }

    number MDP::getHorizon() const
    {
        return this->horizon_;
    }

    std::shared_ptr<Distribution<std::shared_ptr<State>>> MDP::getStartDistribution() const
    {
        return this->start_distrib_;
    }

    std::set<std::shared_ptr<State>> MDP::getReachableStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    {
        return this->state_dynamics_->getReachableStates(state, action, t);
    }
    std::shared_ptr<StateDynamicsInterface> MDP::getStateDynamics() const
    {
        return this->state_dynamics_;
    }

    std::shared_ptr<RewardInterface> MDP::getReward() const
    {
        return this->reward_;
    }

    double MDP::getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    {
        return this->reward_->getReward(state, action, t);
    }

    double MDP::getMinReward(number t) const
    {
        return this->reward_->getMinReward(t);
    }

    double MDP::getMaxReward(number t) const
    {
        return this->reward_->getMaxReward(t);
    }

    double MDP::getTransitionProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t) const
    {
        return this->state_dynamics_->getTransitionProbability(state, action, next_state, t);
    }

    std::shared_ptr<Space> MDP::getActionSpace(number) const
    {
        return this->action_space_;
    }

    std::shared_ptr<Space> MDP::getStateSpace(number) const
    {
        return this->state_space_;
    }
}