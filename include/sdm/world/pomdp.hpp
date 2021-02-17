/*=============================================================================
  Copyright (c) 2020 David Albert
==============================================================================*/
#pragma once

#include <vector>
#include <iostream>

#include <sdm/types.hpp>

#include <sdm/world/decpomdp.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/reward.hpp>
#include <sdm/core/state_dynamics.hpp>
#include <sdm/core/observation_dynamics.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>

namespace sdm
{
    class POMDP : public DecPOMDP
    {

    public:
        POMDP();

        POMDP(DecPOMDP &decpomdp);

        //! \param filename name of the file describing the POMDP
        POMDP(std::string &filename);

        /**
         * @brief   Construct a new POMDP object
		 * @param   state_sp the state space
		 * @param   agent_sp the agent space
		 * @param   action_sp the multi action space
		 * @param   obs_sp the multi observation space
		 * @param   s_dyn state dynamics
		 * @param   o_dyn observation dynamics
		 * @param   rew reward function common to all agents
		 * @param   start_distrib start distribution (optional)
         * 
         */
        POMDP(DiscreteSpace<number> state_sp, MultiDiscreteSpace<number> action_sp, MultiDiscreteSpace<number> obs_sp,
                       StateDynamics s_dyn, ObservationDynamics o_dyn, Reward rew, Vector start_distrib);
    };
} // namespace sdm
