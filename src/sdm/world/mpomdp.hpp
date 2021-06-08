/**
 * @file discrete_pomdp.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief The file that contains the POMDP class.
 * @version 1.0
 * @date 02/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>
#include <sdm/core/dynamics/observation_dynamics_interface.hpp>

#include <sdm/world/mmdp.hpp>
#include <sdm/world/pomdp.hpp>
#include <sdm/world/base/mpomdp_interface.hpp>

namespace sdm
{
    /**
     * @brief The class for Discrete Partially Observable Markov Decision Processes. 
     */
    class MPOMDP : public MPOMDPInterface, public POMDP, public MMDP
    {
    public:
        MPOMDP(double horizon,
               double discount,
               const std::shared_ptr<Space> &state_space,
               const std::shared_ptr<Space> &action_space,
               const std::shared_ptr<Space> &obs_space,
               const std::shared_ptr<RewardInterface> &reward,
               const std::shared_ptr<StateDynamicsInterface> &state_dynamics,
               const std::shared_ptr<ObservationDynamicsInterface> &obs_dynamics,
               const std::shared_ptr<Distribution<std::shared_ptr<State>>> &start_distrib);

        /**
         * @brief Get ths observation space of agent i at timestep t.
         * 
         * @param agent_id the identifier of the agent 
         * @param t the timestep
         * @return the observation space
         */
        virtual std::shared_ptr<Space> getObservationSpace(number agent_id, number t) const;

        /**
         * @brief Get ths observation space at timestep t.
         * 
         * @param t the timestep
         * @return the observation space
         */
        virtual std::shared_ptr<Space> getObservationSpace(number t) const;
    };
} // namespace sdm
