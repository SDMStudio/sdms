#include <random>

#include <sdm/world/decision_process.hpp>
#include <sdm/common.hpp>

namespace sdm
{
    DecisionProcess::DecisionProcess() {}

    // DecisionProcess::DecisionProcess(number n_state, number n_agent,  std::vector<number> &act_space)
    //     : StochasticProcess(n_state),
    //       agent_space_(n_agent), action_space_(act_space)
    // {
    //     this->s_dynamics_ = StateDynamics(this->getNumJActions(), this->getNumStates());
    //     for (number i = 0; i < this->getNumAgents(); i++)
    //     {
    //         this->rew_.push_back(Reward(this->getNumJActions(), this->getNumStates()));
    //     }
    // }

    // DecisionProcess::DecisionProcess(number n_state, number n_agent,  std::vector<number> &act_space,  Vector &start_distrib)
    //     : DecisionProcess(n_state, n_agent, act_space)

    // {
    //     this->setStartDistrib(start_distrib);
    // }

    DecisionProcess::DecisionProcess(DiscreteSpace<number> state_sp, DiscreteSpace<number> agent_sp, MultiDiscreteSpace<number> action_sp)
        : StochasticProcess(state_sp),
          agent_space_(agent_sp), action_space_(action_sp), s_dynamics_(action_sp.getNumJointItems(), state_sp.getNumItems())
    {

        for (number i = 0; i < this->getNumAgents(); i++)
        {
            this->rew_.push_back(Reward(action_sp.getNumJointItems(), state_sp.getNumItems()));
        }
    }

    DecisionProcess::DecisionProcess(DiscreteSpace<number> state_sp, DiscreteSpace<number> agent_sp, MultiDiscreteSpace<number> action_sp, Vector start_distrib)
        : DecisionProcess(state_sp, agent_sp, action_sp)
    {
        this->setStartDistrib(start_distrib);
    }

    DecisionProcess::DecisionProcess(DiscreteSpace<number> state_sp, DiscreteSpace<number> agent_sp, MultiDiscreteSpace<number> action_sp,
                                     StateDynamics s_dyn, std::vector<Reward> rews, Vector start_distrib)
        : StochasticProcess(state_sp, start_distrib), agent_space_(agent_sp), action_space_(action_sp)
    {
        this->s_dynamics_ = s_dyn;
        this->rew_ = rews;
    }

    void DecisionProcess::setFileName(std::string filename)
    {
        this->filename = filename;
    }

    std::string DecisionProcess::getFileName()
    {
        return this->filename;
    }

    bool DecisionProcess::getCriterion() 
    {
        return criterion;
    }

    void DecisionProcess::setCriterion(bool criterion)
    {
        this->criterion = (Criterion)criterion;
    }

    std::vector<double> DecisionProcess::getCost(number state, number jaction)
    {
        std::vector<double> costs;
        for (number ag = 0; ag < this->getNumAgents(); ag++)
        {
            double cost = std::abs((this->rew_[ag].getMinReward() - this->rew_[ag].getReward(state, jaction)) / (this->rew_[ag].getMaxReward() - this->rew_[ag].getMinReward()));
            costs.push_back(cost);
        }
        return costs;
    }

    std::vector<double> DecisionProcess::getCost(number state, std::vector<number> jaction) 
    {
        return this->getCost(state, this->action_space_.joint2single(jaction));
    }

    double DecisionProcess::getBound() 
    {
        return this->bound;
    }

    void DecisionProcess::setBound(double bound)
    {
        this->bound = std::min(1.0 / (bound * (1.0 - this->discount)), 1.0);
    }

    double DecisionProcess::getDiscount() 
    {
        return discount;
    }

    void DecisionProcess::setDiscount(double discount)
    {
        this->discount = discount;
    }

    void DecisionProcess::setPlanningHorizon(number planning_horizon)
    {
        this->planning_horizon = planning_horizon;
    }

    number DecisionProcess::getPlanningHorizon() 
    {
        return this->planning_horizon;
    }

    StateDynamics &DecisionProcess::getStateDynamics()
    {
        return this->s_dynamics_;
    }

    DiscreteSpace<number> &DecisionProcess::getAgentSpace()
    {
        return this->agent_space_;
    }

    // ACTION SPACE
    MultiDiscreteSpace<number> &DecisionProcess::getActionSpace()
    {
        return this->action_space_;
    }

    number DecisionProcess::getNumJActions()
    {
        return this->getActionSpace().getNumJointItems();
    }

    number DecisionProcess::getNumActions(number ag_id)
    {
        return this->getActionSpace().getSpace(ag_id)->getNumItems();
    }

    std::vector<number> DecisionProcess::getNumActions()
    {
        std::vector<number> v;
        for (number i = 0; i < this->getActionSpace().getNumSpaces(); i++)
        {
            v.push_back(this->getNumActions(i));
        }
        return v;
    }

    // AGENT SPACE
    number DecisionProcess::getNumAgents()
    {
        return this->agent_space_.getNumItems();
    }

    // TRANSITIONS

    double DecisionProcess::getTransitionProba(number cstate, number jaction, number nstate)
    {
        return this->s_dynamics_.getTransitionProbability(cstate, jaction, nstate);
    }

    double DecisionProcess::getTransitionProba(number cstate, std::vector<number> jaction, number nstate)
    {
        return this->getTransitionProba(cstate, this->action_space_.joint2single(jaction), nstate);
    }

    void DecisionProcess::nextState(number jaction)
    {
        std::vector<number> v_proba;
        for (number s = 0; s < this->getNumStates(); s++)
        {
            v_proba.push_back(this->s_dynamics_.getTransitions(jaction)(this->getInternalState(), s));
        }
        this->setInternalState(std::discrete_distribution<number>(v_proba.begin(), v_proba.end())(common::global_urng()));
    }

    void DecisionProcess::nextState(std::vector<number> jaction)
    {
        this->nextState(this->action_space_.joint2single(jaction));
    }

    // REWARD

    std::vector<Reward> &DecisionProcess::getRewards()
    {
        return this->rew_;
    }

    double DecisionProcess::getReward(number state, number jaction, number ag_id)
    {
        return this->rew_[ag_id].getReward(state, jaction);
    }

    double DecisionProcess::getReward(number state, std::vector<number> jaction, number ag_id)
    {
        return this->getReward(state, this->action_space_.joint2single(jaction), ag_id);
    }

    std::vector<double> DecisionProcess::getRewards(number state, number jaction)
    {
        std::vector<double> v_rews;
        for (auto r : this->rew_)
        {
            v_rews.push_back(r.getReward(state, jaction));
        }
        return v_rews;
    }

    std::vector<double> DecisionProcess::getRewards(number state, std::vector<number> jaction)
    {
        return this->getRewards(state, this->action_space_.joint2single(jaction));
    }

} // namespace sdm