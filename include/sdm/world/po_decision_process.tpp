#include <sdm/world/po_decision_process.hpp>

namespace sdm
{
    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TObsDynamics, typename TReward, typename TDistrib>
    PartiallyObservableDecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TObsDynamics, TReward, TDistrib>::PartiallyObservableDecisionProcess()
    {
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TObsDynamics, typename TReward, typename TDistrib>
    PartiallyObservableDecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TObsDynamics, TReward, TDistrib>::PartiallyObservableDecisionProcess(
        std::shared_ptr<TStateSpace> state_sp,
        std::shared_ptr<TActionSpace> action_sp,
        std::shared_ptr<TObsSpace> obs_sp,
        std::shared_ptr<TStateDynamics> state_dyn,
        std::shared_ptr<TObsDynamics> obs_dyn,
        std::shared_ptr<TReward> reward_f,
        TDistrib start_distrib,
        number planning_horizon,
        double discount,
        Criterion criterion)
        : StochasticProcessBase<TStateSpace, TDistrib>(state_sp, start_distrib),
          PartiallyObservableProcessBase<TStateSpace, TObsSpace, TDistrib>(state_sp, obs_sp, start_distrib),
          DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib>(state_sp, action_sp, obs_sp, state_dyn, reward_f, start_distrib, planning_horizon, discount, criterion),
          obs_dynamics_(obs_dyn)
    {
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TObsDynamics, typename TReward, typename TDistrib>
    std::shared_ptr<TObsDynamics> PartiallyObservableDecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TObsDynamics, TReward, TDistrib>::getObsDynamics() const
    {
        return this->obs_dynamics_;
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TObsDynamics, typename TReward, typename TDistrib>
    void PartiallyObservableDecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TObsDynamics, TReward, TDistrib>::setObsDynamics(std::shared_ptr<TObsDynamics> obs_dyn)
    {
        this->obs_dynamics_ = obs_dyn;
    }
}