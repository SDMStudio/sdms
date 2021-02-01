/*=============================================================================
  Copyright (c) 2020 David Albert
==============================================================================*/
#pragma once

#include <vector>

#include <sdm/types.hpp>

#include <sdm/world/posg.hpp>
#include <sdm/world/decpomdp.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/reward.hpp>
#include <sdm/core/state_dynamics.hpp>
#include <sdm/core/observation_dynamics.hpp>

//!
//! \file     zsposg.hpp
//! \author   David Albert
//! \brief    Zero-Sum Partially Observable Stochastic Game class
//! \version  1.0
//! \date     24 novembre 2020
//!
//! This class provide useful members for all stochastic processes.

//! \namespace  sdm
//! \brief Namespace grouping all tools required for sequential decision making.
namespace sdm
{
  //! \class  ZSPOSG
  class ZSPOSG : public POSG
  {
  public:
    ZSPOSG();

    ZSPOSG(const DecPOMDP &);

    //! \param    state_sp the state space
    //! \param    agent_sp the agent space
    //! \param    action_sp the multi action space
    //! \param    obs_sp the multi observation space
    //! \param    s_dyn state dynamics
    //! \param    o_dyn observation dynamics
    //! \param    rew reward function common to all agents
    //! \param    start_distrib start distribution (optional)
    ZSPOSG(const DiscreteSpace<number> &, const DiscreteSpace<number> &, const MultiDiscreteSpace<number> &, const MultiDiscreteSpace<number> &,
           const StateDynamics &, const ObservationDynamics &, const Reward &, const Vector &);

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
  };

  typedef ZSPOSG ZeroSumPOSG;
} // namespace sdm
