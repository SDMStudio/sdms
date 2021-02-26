#include <random>

#include <sdm/world/decision_process.hpp>
#include <sdm/common.hpp>

namespace sdm
{
    template <typename TStateSpace, typename TActionSpace, typename TStateDynamics, typename TReward, typename TDistrib>
    DecisionProcess<TStateSpace, TActionSpace, TStateDynamics, TReward, TDistrib>::DecisionProcess()
    {
    }

    template <typename TStateSpace, typename TActionSpace, typename TStateDynamics, typename TReward, typename TDistrib>
    DecisionProcess<TStateSpace, TActionSpace, TStateDynamics, TReward, TDistrib>::DecisionProcess(std::shared_ptr<TStateSpace> state_sp, std::shared_ptr<TActionSpace> action_sp)
        : DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>(state_sp, action_sp)
    {
    }

    template <typename TStateSpace, typename TActionSpace, typename TStateDynamics, typename TReward, typename TDistrib>
    DecisionProcess<TStateSpace, TActionSpace, TStateDynamics, TReward, TDistrib>::DecisionProcess(std::shared_ptr<TStateSpace> state_sp, std::shared_ptr<TActionSpace> action_sp, TDistrib start_distrib)
        : DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>(state_sp, action_sp, start_distrib)
    {
    }

    template <typename TStateSpace, typename TActionSpace, typename TStateDynamics, typename TReward, typename TDistrib>
    DecisionProcess<TStateSpace, TActionSpace, TStateDynamics, TReward, TDistrib>::DecisionProcess(std::shared_ptr<TStateSpace> state_sp,
                                                                                                   std::shared_ptr<TActionSpace> action_sp,
                                                                                                   std::shared_ptr<TStateDynamics> state_dyn,
                                                                                                   std::shared_ptr<TReward> reward_fct,
                                                                                                   TDistrib start_distrib,
                                                                                                   number planning_horizon,
                                                                                                   double discount,
                                                                                                   Criterion criterion)
        : DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>(state_sp, action_sp, start_distrib, planning_horizon, discount, criterion),
          state_dynamics_(state_dyn), reward_function_(reward_fct)
    {
    }

    template <typename TStateSpace, typename TActionSpace, typename TStateDynamics, typename TReward, typename TDistrib>
    std::shared_ptr<TStateDynamics> DecisionProcess<TStateSpace, TActionSpace, TStateDynamics, TReward, TDistrib>::getStateDynamics() const
    {
        return this->state_dynamics_;
    }

    template <typename TStateSpace, typename TActionSpace, typename TStateDynamics, typename TReward, typename TDistrib>
    void DecisionProcess<TStateSpace, TActionSpace, TStateDynamics, TReward, TDistrib>::setStateDynamics(std::shared_ptr<TStateDynamics> state_dyn)
    {
        this->state_dynamics_ = state_dyn;
    }

    template <typename TStateSpace, typename TActionSpace, typename TStateDynamics, typename TReward, typename TDistrib>
    std::shared_ptr<TReward> DecisionProcess<TStateSpace, TActionSpace, TStateDynamics, TReward, TDistrib>::getReward() const
    {
        return this->reward_function_;
    }

    template <typename TStateSpace, typename TActionSpace, typename TStateDynamics, typename TReward, typename TDistrib>
    void DecisionProcess<TStateSpace, TActionSpace, TStateDynamics, TReward, TDistrib>::setReward(std::shared_ptr<TReward> reward_function)
    {
        this->reward_function_ = reward_function;
    }

    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // DecisionProcess<TStateSpace, TActionSpace, TDistrib>::DecisionProcess() {}

    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // DecisionProcess<TStateSpace, TActionSpace, TDistrib>::DecisionProcess(number n_state, number n_agent,  std::vector<number> &act_space)
    //     : StochasticProcess(n_state),
    //       agent_space_(n_agent), action_space_(act_space)
    // {
    //     this->s_dynamics_ = StateDynamics(this->getNumJActions(), this->getNumStates());
    //     for (number i = 0; i < this->getNumAgents(); i++)
    //     {
    //         this->rew_.push_back(Reward(this->getNumJActions(), this->getNumStates()));
    //     }
    // }

    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // DecisionProcess<TStateSpace, TActionSpace, TDistrib>::DecisionProcess(number n_state, number n_agent,  std::vector<number> &act_space,  Vector &start_distrib)
    //     : DecisionProcess(n_state, n_agent, act_space)

    // {
    //     this->setStartDistrib(start_distrib);
    // }

    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // DecisionProcess<TStateSpace, TActionSpace, TDistrib>::DecisionProcess(DiscreteSpace<number> state_sp, DiscreteSpace<number> agent_sp, MultiDiscreteSpace<number> action_sp)
    //     : StochasticProcess(state_sp),
    //       agent_space_(agent_sp), action_space_(action_sp), s_dynamics_(action_sp.getNumJointItems(), state_sp.getNumItems())
    // {

    //     for (number i = 0; i < this->getNumAgents(); i++)
    //     {
    //         this->rew_.push_back(Reward(action_sp.getNumJointItems(), state_sp.getNumItems()));
    //     }
    // }

    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // DecisionProcess<TStateSpace, TActionSpace, TDistrib>::DecisionProcess(DiscreteSpace<number> state_sp, DiscreteSpace<number> agent_sp, MultiDiscreteSpace<number> action_sp, Vector start_distrib)
    //     : DecisionProcess(state_sp, agent_sp, action_sp)
    // {
    //     this->setStartDistrib(start_distrib);
    // }

    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // DecisionProcess<TStateSpace, TActionSpace, TDistrib>::DecisionProcess(TStateSpace state_sp, TActionSpace action_sp,
    //                                                                       StateDynamics<state_type, action_type, TDistrib> s_dyn, Reward *rews, TDistrib start_distrib)
    //     : StochasticProcess(state_sp, start_distrib), action_space_(action_sp)
    // {
    //     this->s_dynamics_ = s_dyn;
    //     this->rew_ = rews;
    // }

    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // void DecisionProcess<TStateSpace, TActionSpace, TDistrib>::setFileName(std::string filename)
    // {
    //     this->filename = filename;
    // }

    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // std::string DecisionProcess<TStateSpace, TActionSpace, TDistrib>::getFileName()
    // {
    //     return this->filename;
    // }

    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // bool DecisionProcess<TStateSpace, TActionSpace, TDistrib>::getCriterion()
    // {
    //     return criterion;
    // }

    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // void DecisionProcess<TStateSpace, TActionSpace, TDistrib>::setCriterion(bool criterion)
    // {
    //     this->criterion = (Criterion)criterion;
    // }

    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // std::vector<double> DecisionProcess<TStateSpace, TActionSpace, TDistrib>::getCost(typename DecisionProcess<TStateSpace, TActionSpace, TDistrib>::state_type state, DecisionProcess<TStateSpace, TActionSpace, TDistrib>::action_type jaction)
    // {
    //     std::vector<double> costs;
    //     for (number ag = 0; ag < this->getNumAgents(); ag++)
    //     {
    //         double cost = std::abs((this->rew_[ag].getMinReward() - this->rew_[ag].getReward(state, jaction)) / (this->rew_[ag].getMaxReward() - this->rew_[ag].getMinReward()));
    //         costs.push_back(cost);
    //     }
    //     return costs;
    // }

    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // std::vector<double> DecisionProcess<TStateSpace, TActionSpace, TDistrib>::getCost(number state, std::vector<number> jaction)
    // {
    //     return this->getCost(state, this->action_space_.joint2single(jaction));
    // }

    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // double DecisionProcess<TStateSpace, TActionSpace, TDistrib>::getBound()
    // {
    //     return this->bound;
    // }

    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // void DecisionProcess<TStateSpace, TActionSpace, TDistrib>::setBound(double bound)
    // {
    //     this->bound = std::min(1.0 / (bound * (1.0 - this->discount)), 1.0);
    // }

    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // double DecisionProcess<TStateSpace, TActionSpace, TDistrib>::getDiscount()
    // {
    //     return discount;
    // }

    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // void DecisionProcess<TStateSpace, TActionSpace, TDistrib>::setDiscount(double discount)
    // {
    //     this->discount = discount;
    // }

    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // void DecisionProcess<TStateSpace, TActionSpace, TDistrib>::setPlanningHorizon(number planning_horizon)
    // {
    //     this->planning_horizon = planning_horizon;
    // }

    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // number DecisionProcess<TStateSpace, TActionSpace, TDistrib>::getPlanningHorizon()
    // {
    //     return this->planning_horizon;
    // }

    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // StateDynamics &DecisionProcess<TStateSpace, TActionSpace, TDistrib>::getStateDynamics()
    // {
    //     return this->s_dynamics_;
    // }

    // // ACTION SPACE
    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // MultiDiscreteSpace<number> &DecisionProcess<TStateSpace, TActionSpace, TDistrib>::getActionSpace()
    // {
    //     return this->action_space_;
    // }

    // // AGENT SPACE
    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // number DecisionProcess<TStateSpace, TActionSpace, TDistrib>::getNumAgents()
    // {
    //     return this->num_agents_;
    // }

    // // TRANSITIONS

    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // double DecisionProcess<TStateSpace, TActionSpace, TDistrib>::getTransitionProba(number cstate, number jaction, number nstate)
    // {
    //     return this->s_dynamics_.getTransitionProbability(cstate, jaction, nstate);
    // }

    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // double DecisionProcess<TStateSpace, TActionSpace, TDistrib>::getTransitionProba(number cstate, std::vector<number> jaction, number nstate)
    // {
    //     return this->getTransitionProba(cstate, this->action_space_.joint2single(jaction), nstate);
    // }

    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // void DecisionProcess<TStateSpace, TActionSpace, TDistrib>::nextState(number jaction)
    // {
    //     std::vector<number> v_proba;
    //     for (number s = 0; s < this->getNumStates(); s++)
    //     {
    //         v_proba.push_back(this->s_dynamics_.getTransitions(jaction)(this->getInternalState(), s));
    //     }
    //     this->setInternalState(std::discrete_distribution<number>(v_proba.begin(), v_proba.end())(common::global_urng()));
    // }

    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // void DecisionProcess<TStateSpace, TActionSpace, TDistrib>::nextState(std::vector<number> jaction)
    // {
    //     this->nextState(this->action_space_.joint2single(jaction));
    // }

    // // REWARD

    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // std::vector<Reward> &DecisionProcess<TStateSpace, TActionSpace, TDistrib>::getRewards()
    // {
    //     return this->rew_;
    // }

    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // double DecisionProcess<TStateSpace, TActionSpace, TDistrib>::getReward(number state, number jaction, number ag_id)
    // {
    //     return this->rew_[ag_id].getReward(state, jaction);
    // }

    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // double DecisionProcess<TStateSpace, TActionSpace, TDistrib>::getReward(number state, std::vector<number> jaction, number ag_id)
    // {
    //     return this->getReward(state, this->action_space_.joint2single(jaction), ag_id);
    // }

    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // std::vector<double> DecisionProcess<TStateSpace, TActionSpace, TDistrib>::getRewards(number state, number jaction)
    // {
    //     std::vector<double> v_rews;
    //     for (auto r : this->rew_)
    //     {
    //         v_rews.push_back(r.getReward(state, jaction));
    //     }
    //     return v_rews;
    // }

    // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
    // std::vector<double> DecisionProcess<TStateSpace, TActionSpace, TDistrib>::getRewards(number state, std::vector<number> jaction)
    // {
    //     return this->getRewards(state, this->action_space_.joint2single(jaction));
    // }

} // namespace sdm