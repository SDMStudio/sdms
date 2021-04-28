#include <random>

#include <sdm/world/decision_process.hpp>
#include <sdm/common.hpp>

namespace sdm
{
    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs>
    DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::DecisionProcess()
    {
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs>
    DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::DecisionProcess(std::shared_ptr<TStateSpace> state_sp, std::shared_ptr<TActionSpace> action_sp)
        : DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>(state_sp, action_sp)
    {
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs>
    DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::DecisionProcess(std::shared_ptr<TStateSpace> state_sp, std::shared_ptr<TActionSpace> action_sp, TDistrib start_distrib)
        : DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>(state_sp, action_sp, start_distrib)
    //   GymInterface<TObsSpace, TActionSpace, std::is_same<typename TReward::value_type, std::vector<double>>::value>(state_sp, action_sp)
    {
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs>
    DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::DecisionProcess(std::shared_ptr<TStateSpace> state_sp,
                                                                                                                            std::shared_ptr<TActionSpace> action_sp,
                                                                                                                            std::shared_ptr<TObsSpace>,
                                                                                                                            std::shared_ptr<TStateDynamics> state_dyn,
                                                                                                                            std::shared_ptr<TReward> reward_fct,
                                                                                                                            TDistrib start_distrib,
                                                                                                                            number planning_horizon,
                                                                                                                            double discount,
                                                                                                                            Criterion criterion,
                                                                                                                            bool setup)
        : DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>(state_sp, action_sp, start_distrib, planning_horizon, discount, criterion),
          //   GymInterface<TObsSpace, TActionSpace, std::is_same<typename TReward::value_type, std::vector<double>>::value>(obs_sp, action_sp),
          state_dynamics_(state_dyn), reward_function_(reward_fct)
    {
        if (setup)
        {
            this->setupDynamicsGenerator();
        }
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs>
    DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::DecisionProcess(std::shared_ptr<TStateSpace> state_sp,
                                                                                                                            std::shared_ptr<TActionSpace> action_sp,
                                                                                                                            std::shared_ptr<TStateDynamics> state_dyn,
                                                                                                                            std::shared_ptr<TReward> reward_fct,
                                                                                                                            TDistrib start_distrib,
                                                                                                                            number planning_horizon,
                                                                                                                            double discount,
                                                                                                                            Criterion criterion,
                                                                                                                            bool setup)
        : DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>(state_sp, action_sp, state_sp, state_dyn, reward_fct, start_distrib, planning_horizon, discount, criterion)
    {
        if (setup)
        {
            this->setupDynamicsGenerator();
        }
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs>
    DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::~DecisionProcess() {}

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs>
    std::shared_ptr<TStateDynamics> DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::getStateDynamics() const
    {
        return this->state_dynamics_;
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs>
    void DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::setStateDynamics(std::shared_ptr<TStateDynamics> state_dyn)
    {
        this->state_dynamics_ = state_dyn;
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs>
    std::shared_ptr<TReward> DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::getReward() const
    {
        return this->reward_function_;
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs>
    double DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::getReward(typename DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::state_type s, typename DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::action_type a)
    {
        return this->getReward()->getReward(s, this->getAction(a));
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs>
    void DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::setReward(std::shared_ptr<TReward> reward_function)
    {
        this->reward_function_ = reward_function;
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs>
    typename DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::observation_type DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::reset()
    {
        return this->resetProcess();
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs>
    template <bool TBool>
    std::enable_if_t<TBool, typename DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::observation_type>
    DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::resetProcess()
    {
        this->ctimestep_ = 0;
        this->setInternalState(this->getStartDistrib()(sdm::common::global_urng()));
        return this->getInternalState();
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs>
    template <bool TBool>
    std::enable_if_t<!TBool, typename DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::observation_type>
    DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::resetProcess()
    {
        this->ctimestep_ = 0;
        this->setInternalState(this->getStartDistrib()(sdm::common::global_urng()));
        return DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::observation_type();
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs>
    TDistrib DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::getNextStateDistrib(DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::state_type cstate, DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::action_type caction)
    {
        return this->dynamics_generator.at(cstate).at(caction);
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs>
    TDistrib DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::getNextStateDistrib(DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::action_type caction)
    {
        return this->getNextStateDistrib(this->getInternalState(), caction);
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs>
    const std::set<typename DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::state_type> &DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::getReachableStates(state_type cstate, action_type caction) const
    {
        return this->reachable_state_space.at(cstate).at(caction);
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs>
    const std::set<typename DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::observation_type> &DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::getReachableObservations(DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::action_type caction, DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::state_type next_state) const
    {
        return this->reachable_observation_space.at(caction).at(next_state);
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs>
    template <bool TBool>
    std::enable_if_t<TBool, typename DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::observation_type> DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::updateState_getObs(DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::action_type a)
    {
        this->setInternalState(this->getNextStateDistrib(a)(sdm::common::global_urng()));
        return this->getInternalState();
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs>
    template <bool TBool>
    std::enable_if_t<TBool, std::tuple<typename DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::observation_type, std::vector<double>, bool>>
    DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::stepProcess(typename DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::action_type a)
    {
        this->ctimestep_++;
        auto rew = this->getReward()->getReward(this->getInternalState(), this->getAction(a));
        auto z = this->updateState_getObs(a);
        bool is_done = (this->getPlanningHorizon() > 0) ? (this->getPlanningHorizon() <= this->ctimestep_) : (1000 <= this->ctimestep_);
        return std::make_tuple(z, std::vector<double>{rew}, is_done);
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs>
    template <bool TBool>
    std::enable_if_t<!TBool, std::tuple<typename DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::observation_type, std::vector<double>, bool>>
        DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::stepProcess(typename DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::action_type)
    {
        throw sdm::exception::Exception("Wrong class function call.");
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs>
    std::tuple<typename DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::observation_type, std::vector<double>, bool>
    DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::step(typename DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::action_type a)
    {
        // std::cout << "In step FO process" << std::endl;
        return this->stepProcess(a);
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs>
    template <bool TBool>
    std::enable_if_t<TBool> DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::setupDynamicsGenerator()
    {
        using state_type = typename DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::state_type;
        // std::cout << "Setup FO process" << std::endl;
        for (auto &x : this->getStateSpace()->getAll())
        {
            this->dynamics_generator.emplace(x, std::unordered_map<action_type, TDistrib>());
            this->reachable_state_space.emplace(x, std::unordered_map<action_type, std::set<state_type>>());

            for (auto &a : this->getActionSpace()->getAll())
            {
                std::vector<double> v;
                this->reachable_state_space[x].emplace(a, std::set<state_type>());

                for (auto &y : this->getStateSpace()->getAll())
                {
                    v.push_back(this->getStateDynamics()->getTransitionProbability(x, this->getAction(a), y));
                    if (this->getStateDynamics()->getTransitionProbability(x, this->getAction(a), y) > 0)
                    {
                        this->reachable_state_space[x][a].insert(y);
                    }
                }

                this->dynamics_generator[x].emplace(a, TDistrib(v.begin(), v.end()));
            }
        }

        // Setup reachable observations
        for (auto &a : this->getActionSpace()->getAll())
        {
            this->reachable_observation_space.emplace(a, std::unordered_map<state_type, std::set<observation_type>>());
            for (auto &x : this->getStateSpace()->getAll())
            {
                this->reachable_observation_space[a].emplace(x, std::set<observation_type>());
                for (auto &y : this->getStateSpace()->getAll())
                {
                    if (this->getStateDynamics()->getTransitionProbability(x, this->getAction(a), y) > 0)
                    {
                        this->reachable_observation_space[a][x].insert(y);
                    }
                }
            }
        }
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs>
    template <bool TBool>
    std::enable_if_t<!TBool> DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::setupDynamicsGenerator()
    {
        throw sdm::exception::NotImplementedException();
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs>
    std::shared_ptr<TActionSpace> DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::getActionSpace() const
    {
        return DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>::getActionSpace();
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs>
    template <bool TBool>
    std::enable_if_t<TBool, number> DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::getNumAgents()
    {
        return this->getActionSpace()->getNumSpaces();
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs>
    template <bool TBool>
    std::enable_if_t<!TBool, number> DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::getNumAgents()
    {
        return 1;
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs>
    template <bool TBool>
    std::enable_if_t<TBool, number> DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::getAction(DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::action_type a)
    {
        return this->getActionSpace()->joint2single(a);
    }

    template <typename TStateSpace, typename TActionSpace, typename TObsSpace, typename TStateDynamics, typename TReward, typename TDistrib, bool is_fully_obs>
    template <bool TBool>
    std::enable_if_t<!TBool, typename DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::action_type> DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::getAction(DecisionProcess<TStateSpace, TActionSpace, TObsSpace, TStateDynamics, TReward, TDistrib, is_fully_obs>::action_type a)
    {
        return a;
    }

} // namespace sdm