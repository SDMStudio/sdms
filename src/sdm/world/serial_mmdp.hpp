/**
 * @file serial_mmdp.hpp
 * @author Jérôme ARJONILLA 
 * @brief Defines the Serial MMDP.
 * @version 0.1
 * @date 17/08/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once

#include <sdm/types.hpp>

#include <sdm/core/state/state.hpp>
#include <sdm/core/state/serial_state.hpp>

#include <sdm/core/action/action.hpp>
#include <sdm/core/distribution.hpp>
#include <sdm/world/base/mmdp_interface.hpp>

#include <sdm/core/space/multi_space.hpp>
#include <sdm/core/space/discrete_space.hpp>

#include <unordered_map>
#include <sdm/core/dynamics/tabular_state_dynamics.hpp>
#include <sdm/core/state/interface/serial_interface.hpp>

namespace sdm
{
    class SerialMMDP : virtual public SerialMMDPInterface
    {
    public:
        SerialMMDP(const std::shared_ptr<MMDPInterface> &mmdp);

        virtual ~SerialMMDP();

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
        bool isLastAgent(number t) const;

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
        std::shared_ptr<Space> getStateSpace(number t = 0) const;

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

        // For Gym Interface

        /**
         * @brief Get the action space.
         * @param observation the observation in consideration
         * @param t time step
         * @return the action space. 
         */
        std::shared_ptr<Space> getActionSpaceAt(const std::shared_ptr<State> &observation, number t);

        /**
         * @brief Reset the environment and return initial observation.
         * @return the initial observation
         */
        std::shared_ptr<State> reset();

        /**
         * @brief Do a step on the environment.
         * @param action the action to execute
         * @return the information produced. Include : next observation, rewards, episode done  
         */
        std::tuple<std::shared_ptr<State>, std::vector<double>, bool> step(std::shared_ptr<Action> action);

        std::tuple<std::shared_ptr<State>, std::vector<double>, bool> step(std::shared_ptr<Action> action, bool increment_timestep);

        void setInternalState(std::shared_ptr<State> state);

        std::shared_ptr<State> getInternalState() const;

        std::shared_ptr<Action> getRandomAction(const std::shared_ptr<State> &, number t);
        
    protected:

        /**
         * @brief The mmpdp associated to the problem
         * 
         */
        std::shared_ptr<MMDPInterface> mmdp_;

        /**
         * @brief Refer to the Serial State Space
         * 
         */
        Joint<std::shared_ptr<DiscreteSpace>> serial_state_space_;

        /**
         * @brief Map (serial state, seial action) to Set of reachable seial states
         */
        // std::unordered_map<std::shared_ptr<State>, std::unordered_map<std::shared_ptr<Action>, std::set<std::shared_ptr<State>>>> reachable_state_space;
        std::shared_ptr<StateDynamicsInterface> state_dynamics_;

        /**
         * @brief Map the joint_action to a precise pointeur of Action
         * 
         */
        std::unordered_map<Joint<std::shared_ptr<Action>>, std::shared_ptr<Action>> map_joint_action_to_pointeur;

        /**
         * @brief Map the serialState to a precise pointeur of State
         * 
         */
        std::unordered_map<SerialState, std::shared_ptr<State>> map_serial_state_to_pointeur;

        std::shared_ptr<Distribution<std::shared_ptr<State>>> distribution_serial;

        /**
         * @brief Initialize Serial State Space
         * 
         */
        void createInitSerialStateSpace();

        /**
         * @brief Initialize "serial_state_space_"
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
        const std::shared_ptr<State> getPointeurState(SerialState &) const;

        void createDistribution();

        /**
         * @brief A serial state is composed by a vector of action, in this function, we add a new action to the current vector of action/
         * 
         * @param const std::shared_ptr<State>& : current serial state
         * @param const std::shared_ptr<Action>& : action to add
         * @return Joint<std::shared_ptr<Action>> : vector of action of the current serial state + the action to add
         */
        Joint<std::shared_ptr<Action>> addNewAction(const std::shared_ptr<State>& state, const std::shared_ptr<Action>& new_action) const;
    };

} // namespace sdm