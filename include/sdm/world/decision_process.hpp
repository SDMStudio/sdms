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
        // DecisionProcess(number, number, std::vector<number> &);
        // DecisionProcess(number, number, std::vector<number> &, Vector &);
        DecisionProcess(DiscreteSpace<number>, DiscreteSpace<number>, MultiDiscreteSpace<number>);
        DecisionProcess(DiscreteSpace<number>, DiscreteSpace<number>, MultiDiscreteSpace<number>, Vector);
        DecisionProcess(DiscreteSpace<number>, DiscreteSpace<number>, MultiDiscreteSpace<number>, StateDynamics, std::vector<Reward>, Vector);

        //! \fn       void setFileName(std::string)
        //! \param   filename
        void setFileName(std::string);

        //! \fn       std::string getFileName()
        //! \return   filename
        std::string getFileName();

        //! \fn       bool getBound()
        //! \brief    Returns the bound
        //! \return   the bound
        double getBound();

        //! \fn       vois setBound(double)
        //! \brief    Set the bound
        //! \param   bound the bound
        void setBound(double);

        //! \fn       bool getCriterion()
        //! \brief    Returns the criterion
        //! \return   bool
        bool getCriterion();

        //! \fn       void setCriterion(bool)
        //! \brief    Sets the criterion
        void setCriterion(bool);

        //! \fn       value getDiscount()
        //! \brief    Returns the discount factor
        //! \return   value
        double getDiscount();

        //! \fn       void setDiscount(value)
        //! \brief    Sets the discount factor
        void setDiscount(double);

        //! \fn       number getPlanningHorizon()
        //! \brief    Returns the planning horizon
        //! \return   the horizon
        number getPlanningHorizon();

        //! \fn       void setPlanningHorizon(number)
        //! \brief    Sets the planning horizon
        void setPlanningHorizon(number);

        /**
         * \brief Get the state dynamics
         */
     StateDynamics &getStateDynamics();

        /**
         * \brief Get transition probability from joint action
         */
        double getTransitionProba(number cstate, std::vector<number> jaction, number state);

        /**
         * \brief Get transition probability from joint action (as a single one)
         */
        double getTransitionProba(number cstate, number jaction, number state);

        /**
         * \brief Transit to next state given a joint action (as a single one)
         */
        void nextState(number jaction);

        /**
         * \brief Transit to next state given a joint action
         */
        void nextState(std::vector<number> jaction);

        std::vector<Reward> &getRewards();

        /**
         * \brief Get transition probability from joint action (as a single one) for all agents
         */
        std::vector<double> getRewards(number state, number jaction);

        /**
         * \brief Get reward from joint action for all agents 
         */
        std::vector<double> getRewards(number state, std::vector<number> jaction);

        /**
         * \fn double getReward(number state, number jaction, number ag_id);
         * \brief Get reward from joint action (as single one) for a specific agent 
         */
        double getReward(number state, number jaction, number ag_id);

        /**
         * \fn double getReward(number state, number jaction, number ag_id);
         * \brief Get reward from joint action for a specific agent 
         */
        double getReward(number state, std::vector<number> jaction, number ag_id);

        /**
         * \fn double getReward(number state, number jaction, number ag_id);
         * \brief Get reward from joint action for all agents
         */
        std::vector<double> getCost(number state, number jaction);

        /**
         * \fn std::vector<double> getReward(number state, std::vector<number> jaction);
         * \brief Get cost from joint action for all agents
         */
        std::vector<double> getCost(number state, std::vector<number> jaction);

        /**
         * \brief Getter for the action spaces
         */
        DiscreteSpace<number> &getAgentSpace();

        /**
         * \brief Getter for the action spaces
         */
        MultiDiscreteSpace<number> &getActionSpace();

        /**
         * \brief Get the number of joint actions
         */
        number getNumJActions();

        /**
         * \brief Get the number of actions for a specific agent
         */
        number getNumActions(number);

        /**
         * \brief Get the number of actions for each agents
         */
        std::vector<number> getNumActions();

        /**
         * \brief Get the number of agents
         */
        number getNumAgents();
    };

    typedef DecisionProcess SG;
    typedef DecisionProcess StochasticGame;
} // namespace sdm
