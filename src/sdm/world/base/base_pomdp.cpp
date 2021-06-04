#include <sdm/world/base_pomdp.hpp>
#include <sdm/parser/parser.hpp>

namespace sdm
{

    BasePOMDP::BasePOMDP(number num_agents,
                         double discount,
                         const std::shared_ptr<Space<std::shared_ptr<State>>> &state_space,
                         const std::shared_ptr<Space<std::shared_ptr<Action>>> &action_space,
                         const std::shared_ptr<Space<std::shared_ptr<Observation>>> &obs_space,
                         const std::shared_ptr<BaseReward> &reward,
                         const std::shared_ptr<StateDynamicsInterface> &state_dynamics,
                         const std::shared_ptr<BaseObservationDynamics> &obs_dynamics) : BaseMDP(num_agents, discount, state_space, action_space, reward, state_dynamics),
                                                                                         obs_space_(obs_space),
                                                                                         obs_dynamics_(obs_dynamics)
    {
    }

    std::set<std::shared_ptr<Observation>> BasePOMDP::getReachableObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    {
    }

    std::vector<std::shared_ptr<Observation>> BasePOMDP::getAllObservations(number t) const
    {
        return this->getObsSpace(t)->getAll();
    }

    std::set<std::shared_ptr<Observation>> BasePOMDP::getObservationSpace(number) const
    {
        return this->obs_space_;
    }

    double BasePOMDP::getObservationProbability(const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const
    {
        return this->obs_dynamics_->getObservationProbability(action, next_state, observation, t);
    }

    double BasePOMDP::getDynamics(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const
    {
        return this->obs_dynamics_->getDynamics(state, action, next_state, observation, t);
    }

}