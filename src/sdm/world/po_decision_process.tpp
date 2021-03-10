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
          DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib>(state_sp, action_sp, obs_sp, state_dyn, reward_f, start_distrib, planning_horizon, discount, criterion),
          PartiallyObservableProcessBase<TStateSpace, TObsSpace, TDistrib>(state_sp, obs_sp, start_distrib),
          obs_dynamics_(obs_dyn)
    {
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TObsDynamics, typename TReward, typename TDistrib>
    PartiallyObservableDecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TObsDynamics, TReward, TDistrib>::PartiallyObservableDecisionProcess(PartiallyObservableDecisionProcess &copy)
        : PartiallyObservableDecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TObsDynamics, TReward, TDistrib>(copy.getStateSpace(),
                                                                                                                                    copy.getActionSpace(),
                                                                                                                                    copy.getObsSpace(),
                                                                                                                                    copy.getStateDynamics(),
                                                                                                                                    copy.getObsDynamics(),
                                                                                                                                    copy.getReward(),
                                                                                                                                    copy.getStartDistrib(),
                                                                                                                                    copy.getPlanningHorizon(),
                                                                                                                                    copy.getDiscount(),
                                                                                                                                    copy.getCriterion())
    {
    }
    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TObsDynamics, typename TReward, typename TDistrib>
    PartiallyObservableDecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TObsDynamics, TReward, TDistrib>::~PartiallyObservableDecisionProcess()
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

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TObsDynamics, typename TReward, typename TDistrib>
    template <bool TBool>
    std::enable_if_t<TBool> PartiallyObservableDecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TObsDynamics, TReward, TDistrib>::setupDynamicsGenerator()
    {
        int i = 0;
        for (auto &y : this->getStateSpace()->getAll())
        {
            for (auto &z : this->getObsSpace()->getAll())
            {
                this->encoding.emplace(i, std::make_pair(y, z));
                i++;
            }
        }

        for (auto &x : this->getStateSpace()->getAll())
        {
            this->dynamics_generator.emplace(x, std::unordered_map<PartiallyObservableDecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TObsDynamics, TReward, TDistrib>::action_type, TDistrib>());
            for (auto &a : this->getActionSpace()->getAll())
            {
                std::vector<double> v;
                for (auto &y : this->getStateSpace()->getAll())
                {
                    for (auto &z : this->getObsSpace()->getAll())
                    {
                        v.push_back(this->getObsDynamics()->getDynamics(x, this->getAction(a), this->getObservation(z), y));
                    }
                }
                this->dynamics_generator[x].emplace(a, TDistrib(v.begin(), v.end()));
            }
        }
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TObsDynamics, typename TReward, typename TDistrib>
    typename PartiallyObservableDecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TObsDynamics, TReward, TDistrib>::observation_type PartiallyObservableDecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TObsDynamics, TReward, TDistrib>::updateState_getObs(PartiallyObservableDecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TObsDynamics, TReward, TDistrib>::action_type a)
    {
        std::cout << "IN PO_PROCESS" << std::endl;
        number state_obs_index = this->getNextStateDistrib(a)(sdm::common::global_urng());
        auto pair_st_obs = this->encoding[state_obs_index];
        this->setInternalState(pair_st_obs.first);
        return pair_st_obs.second;
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TObsDynamics, typename TReward, typename TDistrib>
    template <bool TBool>
    std::enable_if_t<!TBool> PartiallyObservableDecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TObsDynamics, TReward, TDistrib>::setupDynamicsGenerator()
    {
        throw sdm::exception::NotImplementedException();
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TObsDynamics, typename TReward, typename TDistrib>
    template <bool TBool>
    std::enable_if_t<TBool, number>
    PartiallyObservableDecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TObsDynamics, TReward, TDistrib>::getObservation(PartiallyObservableDecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TObsDynamics, TReward, TDistrib>::observation_type o)
    {
        return this->getObsSpace()->joint2single(o);
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TObsDynamics, typename TReward, typename TDistrib>
    template <bool TBool>
    std::enable_if_t<!TBool,
                     typename PartiallyObservableDecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TObsDynamics, TReward, TDistrib>::observation_type>
    PartiallyObservableDecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TObsDynamics, TReward, TDistrib>::getObservation(PartiallyObservableDecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TObsDynamics, TReward, TDistrib>::observation_type o)
    {
        return o;
    }

}