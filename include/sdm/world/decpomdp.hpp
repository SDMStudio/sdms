/*=============================================================================
  Copyright (c) 2020 David Albert
==============================================================================*/
#pragma once

#include <vector>
#include <iostream>

#include <sdm/types.hpp>

#include <sdm/world/posg.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/reward.hpp>
#include <sdm/core/state_dynamics.hpp>
#include <sdm/core/observation_dynamics.hpp>

//!
//! \file     decpomdp.hpp
//! \author   David Albert
//! \brief    Decentralized POMDP class
//! \version  1.0
//! \date     24 novembre 2020
//!
//! This class provide useful members for all stochastic processes.

//! \namespace  sdm
//! \brief Namespace grouping all tools required for sequential decision making.
namespace sdm
{
	//! \class  DecPOMDP
	class DecPOMDP : public POSG
	{

	public:
		DecPOMDP();

		DecPOMDP(POSG &);

		//! \param filename name of the file describing the DecPOMDP
		DecPOMDP(std::string &filename);

		//! \param    num_states number of states
		//! \param    num_agents number of agents
		DecPOMDP(number, number);

		//! \param    num_states number of states
		//! \param    num_agents number of agents
		//! \param    num_actions number of actions for each agent
		//! \param    num_observations number of observations for each agent
		DecPOMDP(number, number, std::vector<number> &, std::vector<number> &);

		//! \param    state_sp the state space
		//! \param    agent_sp the agent space
		DecPOMDP(DiscreteSpace<number> &, DiscreteSpace<number> &);

		//! \param    state_sp the state space
		//! \param    agent_sp the agent space
		//! \param    action_sp the multi action space
		//! \param    obs_sp the multi observation space
		//! \param    s_dyn state dynamics
		//! \param    o_dyn observation dynamics
		//! \param    rew reward function common to all agents
		//! \param    start_distrib start distribution (optional)
		DecPOMDP(DiscreteSpace<number>, DiscreteSpace<number>, MultiDiscreteSpace<number>, MultiDiscreteSpace<number>,
				 StateDynamics, ObservationDynamics, Reward, Vector);

		/**
         * \fn Reward getReward();
         * \brief Get reward function
         */
		Reward &getReward();

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
         * \fn double getReward(number state, number jaction);
         * \brief Get reward from joint action 
         */
		double getReward(number state, number jaction);

		/**
         * \fn double getReward(number state, number jaction);
         * \brief Get reward from joint action  
         */
		double getReward(number state, std::vector<number> jaction);

		/**
         * \fn double getCost(number state, number jaction);
         * \brief Get cost from joint action 
         */
		double getCost(number state, number jaction);

		/**
         * \fn double getCost(number state, number jaction);
         * \brief Get cost from joint action  
         */
		double getCost(number state, std::vector<number> jaction);

		// virtual state init();

		// virtual void execute(action, feedback *);
	};

	typedef DecPOMDP DecentralizedPOMDP;
} // namespace sdm