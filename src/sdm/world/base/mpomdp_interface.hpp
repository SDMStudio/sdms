/**
 * @file discrete_mdp.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief The file that contains the MDP class.
 * @version 1.0
 * @date 02/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/core/space/space.hpp>
#include <sdm/world/base/mmdp_interface.hpp>
#include <sdm/world/base/pomdp_interface.hpp>

namespace sdm
{
    /**
     * @brief The class for Discrete Markov Decision Processes. 
     * 
     */
    class MPOMDPInterface : virtual public MMDPInterface,
                            virtual public POMDPInterface
    {
    public:

        /**
         * @brief Get ths observation space of agent i at timestep t.
         * 
         * @param agent_id the identifier of the agent 
         * @param t the timestep
         * @return the observation space
         */
        virtual std::shared_ptr<Space> getObservationSpace(number agent_id, number t) const = 0;
    };

} // namespace sdm