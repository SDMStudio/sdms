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

#include <sdm/core/state/state.hpp>
#include <sdm/core/state/serialized_state.hpp>

#include <sdm/core/action/action.hpp>
#include <sdm/core/distribution.hpp>
#include <sdm/world/base/mmdp_interface.hpp>

#include <sdm/core/space/multi_space.hpp>
#include <sdm/core/space/discrete_space.hpp>

#include <unordered_map>
#include <sdm/core/dynamics/tabular_state_dynamics.hpp>

namespace sdm
{
    class SerializedMMDP : virtual public MMDPInterface
    {
    public:
        SerializedMMDP(const std::shared_ptr<MMDPInterface> &mmdp);

        virtual ~SerializedMMDP();

        /**
         * @brief Get the number of agents
         * 
         * @return the number of agents
         */
        number getNumAgents() const;

        /**
         * @brief Get the identifier of the current agent.
         * 
         * @param t the timestep
         * @return number the agent id
         */
        number getAgentId(number t) const;

        /**
         * @brief 
         * 
         * @param t 
         * @return true 
         * @return false 
         */
        bool isLastAgent(number t) const ;

        /**
         * @brief Get the discount factor at timestep t.
         * 
         * @param t the timestep
         * @return the discount factor
         */
        double getDiscount(number t = 0) const;

        /**
         * @brief Get the number of agents
         * 
         * @return the number of agents
         */
        number getHorizon() const;

        /**
         * @brief Get the initial distribution over states.
         * 
         * @return the initial distribution over states
         */
        std::shared_ptr<Distribution<std::shared_ptr<State>>> getStartDistribution() const;

        /**
         * @brief Get all states
         * 
         * @return the set of states 
         */
        std::shared_ptr<Space> getStateSpace(number t =0) const;

        /**
         * @brief Get the reachable next states
         * 
         * @param state the state
         * @param action the action
         * @return the set of reachable states
         */
        std::set<std::shared_ptr<State>> getReachableStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0) const;

        /**
         * @brief Get all actions at a specific time step
         * 
         * @return the set of actions 
         */
        std::shared_ptr<Space> getActionSpace(number t = 0) const;

        /**
         * @brief Get the Action Space at a specific time step and a precise agent
         * 
         * @param agent_id 
         * @param t 
         * @return std::shared_ptr<Space> 
         */
        std::shared_ptr<Space> getActionSpace(number agent_id, number t ) const;


        /**
         * @brief Get the reward
         * 
         * @param state 
         * @param action 
         * @param t 
         * @return double 
         */
        double getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0) const;

        double getMinReward(number t = 0) const;
        
        double getMaxReward(number t = 0) const;
        /**
         * @brief Get the Transition Probability object
         * 
         * @param state 
         * @param action 
         * @param next_state 
         * @param t 
         * @return double 
         */
        double getTransitionProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t = 0) const;

    protected:
        std::shared_ptr<MMDPInterface> mmdp_;
        
        /**
         * @brief Refer to the Serialized State Space
         * 
         */
        Joint<std::shared_ptr<DiscreteSpace>> serialized_state_space_;

        /**
         * @brief Map (serial state, seial action) to Set of reachable seial states
         */
        // std::unordered_map<std::shared_ptr<State>, std::unordered_map<std::shared_ptr<Action>, std::set<std::shared_ptr<State>>>> reachable_state_space;
        std::shared_ptr<StateDynamicsInterface> state_dynamics_;

        /**
         * @brief Map the joint_action to a precise pointeur
         * 
         */
        std::unordered_map<Joint<std::shared_ptr<Action>>, std::shared_ptr<Action>> map_joint_action_to_pointeur;

        std::unordered_map<SerializedState, std::shared_ptr<State>> map_serialized_state_to_pointeur;

        /**
         * @brief Initialize Serial State Space
         * 
         */
        void createInitSerializedStateSpace();

        /**
         * @brief Initialize "serialized_state_space_"
         * 
         */
        void createInitReachableStateSpace();

        /**
         * @brief Get the Pointeur object of a precise Joint Action
         * 
         * @return std::shared_ptr<Joint<std::shared_ptr<Action>>> 
         */
        const std::shared_ptr<Action> getPointeurJointAction(Joint<std::shared_ptr<Action>> &) const;

        void setJointActionToPointeur(std::vector<Joint<std::shared_ptr<Action>>>);

        /**
         * @brief Get the Pointeur object of a precise Joint Action
         * 
         * @return std::shared_ptr<Joint<std::shared_ptr<Action>>> 
         */
        const std::shared_ptr<State> getPointeurState(SerializedState &) const;

    };

} // namespace sdm