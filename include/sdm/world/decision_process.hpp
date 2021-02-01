/*=============================================================================
  Copyright (c) 2020 David Albert
==============================================================================*/
#pragma once

#include <vector>

#include <sdm/types.hpp>

#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/state_dynamics.hpp>
#include <sdm/core/reward.hpp>

#include <sdm/public/world.hpp>
#include <sdm/world/stochastic_process.hpp>

//!
//! \file     decision_process.hpp
//! \author   David Albert
//! \brief    Decision processes class
//! \version  1.0
//! \date     24 novembre 2020
//!
//! This class provide a way to instantiate decision processes.

//! \namespace  sdm
//! \brief Namespace grouping all tools required for sequential decision making.
namespace sdm
{
    //! \class  DecisionProcess
    //! \brief Decision process
    class DecisionProcess : public virtual StochasticProcess, public World
    {
    protected:
        //! \brief Space of agents (contain number of agents and their names).
        DiscreteSpace<number> agent_space_;

        //! \brief Action space for each agent.
        MultiDiscreteSpace<number> action_space_;

        //! \brief State dynamics.
        StateDynamics s_dynamics_;

        //! \brief Reward functions.
        std::vector<Reward> rew_;

        //! \brief type of optimization problem, e.g., reward maximazation or cost minimization.
        Criterion criterion = Criterion::REW_MAX;

        //! \brief factor used to discount rewards (respectively costs) in the future.
        double discount = 1.0, bound;

        //! \brief planning horizon
        number planning_horizon = 0;

        //! \brief name of the file that generates the environment
        std::string filename;

    public:
        DecisionProcess();
        // DecisionProcess(number, number, const std::vector<number> &);
        // DecisionProcess(number, number, const std::vector<number> &, const Vector &);
        DecisionProcess(const DiscreteSpace<number> &, const DiscreteSpace<number> &, const MultiDiscreteSpace<number> &);
        DecisionProcess(const DiscreteSpace<number> &, const DiscreteSpace<number> &, const MultiDiscreteSpace<number> &, const Vector &);
        DecisionProcess(const DiscreteSpace<number> &, const DiscreteSpace<number> &, const MultiDiscreteSpace<number> &, const StateDynamics &, const std::vector<Reward> &, const Vector &);

        //! \fn       void setFileName(std::string)
        //! \param   filename
        void setFileName(std::string);

        //! \fn       std::string getFileName()
        //! \return   filename
        std::string getFileName();

        //! \fn       bool getBound() const
        //! \brief    Returns the bound
        //! \return   the bound
        double getBound() const;

        //! \fn       vois setBound(double)
        //! \brief    Set the bound
        //! \param   bound the bound
        void setBound(double);

        //! \fn       bool getCriterion() const
        //! \brief    Returns the criterion
        //! \return   bool
        bool getCriterion() const;

        //! \fn       void setCriterion(bool)
        //! \brief    Sets the criterion
        void setCriterion(bool);

        //! \fn       value getDiscount() const
        //! \brief    Returns the discount factor
        //! \return   value
        double getDiscount() const;

        //! \fn       void setDiscount(value)
        //! \brief    Sets the discount factor
        void setDiscount(double);

        //! \fn       number getPlanningHorizon() const
        //! \brief    Returns the planning horizon
        //! \return   the horizon
        number getPlanningHorizon() const;

        //! \fn       void setPlanningHorizon(number)
        //! \brief    Sets the planning horizon
        void setPlanningHorizon(number);

        /**
         * \brief Get the state dynamics
         */
        const StateDynamics &getStateDynamics() const;

        /**
         * \brief Get transition probability from joint action
         */
        double getTransitionProba(number cstate, std::vector<number> jaction, number state) const;

        /**
         * \brief Get transition probability from joint action (as a single one)
         */
        double getTransitionProba(number cstate, number jaction, number state) const;

        /**
         * \brief Transit to next state given a joint action (as a single one)
         */
        void nextState(number jaction);

        /**
         * \brief Transit to next state given a joint action
         */
        void nextState(std::vector<number> jaction);

        const std::vector<Reward> &getRewards() const;

        /**
         * \brief Get transition probability from joint action (as a single one) for all agents
         */
        std::vector<double> getRewards(number state, number jaction) const;

        /**
         * \brief Get reward from joint action for all agents 
         */
        std::vector<double> getRewards(number state, std::vector<number> jaction) const;

        /**
         * \fn double getReward(number state, number jaction, number ag_id);
         * \brief Get reward from joint action (as single one) for a specific agent 
         */
        double getReward(number state, number jaction, number ag_id) const;

        /**
         * \fn double getReward(number state, number jaction, number ag_id);
         * \brief Get reward from joint action for a specific agent 
         */
        double getReward(number state, std::vector<number> jaction, number ag_id) const;

        /**
         * \fn double getReward(number state, number jaction, number ag_id);
         * \brief Get reward from joint action for all agents
         */
        std::vector<double> getCost(number state, number jaction) const;

        /**
         * \fn std::vector<double> getReward(number state, std::vector<number> jaction);
         * \brief Get cost from joint action for all agents
         */
        std::vector<double> getCost(number state, std::vector<number> jaction) const;

        /**
         * \brief Getter for the action spaces
         */
        const DiscreteSpace<number> &getAgentSpace() const;

        /**
         * \brief Getter for the action spaces
         */
        const MultiDiscreteSpace<number> &getActionSpace() const;

        /**
         * \brief Get the number of joint actions
         */
        number getNumJActions() const;

        /**
         * \brief Get the number of actions for a specific agent
         */
        number getNumActions(number) const;

        /**
         * \brief Get the number of actions for each agents
         */
        std::vector<number> getNumActions() const;

        /**
         * \brief Get the number of agents
         */
        number getNumAgents() const;

    };

    typedef DecisionProcess SG;
    typedef DecisionProcess StochasticGame;
} // namespace sdm
