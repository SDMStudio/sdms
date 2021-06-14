#include <sdm/world/pomdp.hpp>

namespace sdm
{

    POMDP::POMDP(const std::shared_ptr<Space> &state_space,
                 const std::shared_ptr<Space> &action_space,
                 const std::shared_ptr<Space> &obs_space,
                 const std::shared_ptr<RewardInterface> &reward,
                 const std::shared_ptr<StateDynamicsInterface> &state_dynamics,
                 const std::shared_ptr<ObservationDynamicsInterface> &obs_dynamics,
                 const std::shared_ptr<Distribution<std::shared_ptr<State>>> &start_distrib,
                 number horizon,
                 double discount,
                 Criterion criterion) : MDP(state_space, action_space, reward, state_dynamics, start_distrib, horizon, discount, criterion),
                                        obs_space_(obs_space),
                                        obs_dynamics_(obs_dynamics)
    {
    }

    std::shared_ptr<Space> POMDP::getObservationSpace(number) const
    {
        return this->obs_space_;
    }

    std::set<std::shared_ptr<Observation>> POMDP::getReachableObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t) const
    {
        return this->obs_dynamics_->getReachableObservations(state, action, next_state, t);
    }

    double POMDP::getObservationProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const
    {
        return this->obs_dynamics_->getObservationProbability(state, action, next_state, observation, t);
    }

    double POMDP::getDynamics(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const
    {
        return this->getTransitionProbability(state, action, next_state, t) * this->getObservationProbability(state, action, next_state, observation, t);
    }

    std::shared_ptr<ObservationDynamicsInterface> POMDP::getObservationDynamics() const
    {
        return this->obs_dynamics_;
    }

}