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
#include <sdm/world/base/mdp_interface.hpp>
#include <sdm/world/base/mdp_interface.hpp>

namespace sdm
{
    /**
     * @brief The class for Discrete Markov Decision Processes. 
     * 
     */
    class MMDPInterface : virtual public MDPInterface
    {
    public:
        /**
         * @brief Get ths action space of agent i at timestep t.
         * 
         * @param agent_id the identifier of the agent 
         * @param t the timestep
         * @return the action space
         */
        virtual std::shared_ptr<Space> getActionSpace(number agent_id, number t) const = 0;

        /**
         * @brief Get ths action space at timestep t.
         * 
         * @param t the timestep
         * @return the action space
         */
        virtual std::shared_ptr<Space> getActionSpace(number t) const = 0;
    };


    class SerialProblemInterface
    {
    public:
        /**
         * @brief Get the identifier of the current agent.
         * 
         * @param t the timestep
         * @return number the agent id
         */
        virtual number getAgentId(number t) const = 0;

        /**
         * @brief 
         * 
         * @param t 
         * @return true 
         * @return false 
         */
        virtual bool isLastAgent(number t) const = 0;
    };


    /**
     * @brief The class for Discrete Markov Decision Processes. 
     * 
     */
    class SerialMMDPInterface : virtual public MMDPInterface, public SerialProblemInterface
    {
    };

} // namespace sdm