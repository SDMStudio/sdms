#pragma once

#include <sdm/types.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/utils/linear_algebra/mapped_matrix.hpp>
#include <sdm/world/serialized_belief_mdp.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/state/serialized_state.hpp>

#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/core/action/det_decision_rule.hpp>

#include <sdm/world/serialized_mmdp_structure.hpp>
#include <sdm/utils/linear_algebra/matrix.hpp>

namespace sdm
{
    /**
     * @brief An Serialized MDP is a subclass of MDP where states are serialized states. 
     * In the general case, a serialized state refers to the whole knowledge that a central planner can have access to take decisions at the time step of an precise agent. 
     * 
     * @tparam state_type refer to the serialized state type
     * @tparam number refer to the number type
     */
    class SerializedMPOMDP : public SerializedMMDPStructure
    {
    public:
        using action_type = number;
        using state_type = SerializedState;
        using observation_type = Joint<number>;

        SerializedMPOMDP();
        SerializedMPOMDP(std::string);
        SerializedMPOMDP(std::shared_ptr<DiscreteDecPOMDP>);

        const std::set<observation_type> &getReachableObservations(state_type, action_type, state_type) const;

        std::shared_ptr<SerializedMMDP> toMDP();
        double getDiscount(number t = 0) const;

        number getNumAgents() const;

        /**
         * @brief Get the corresponding Belief Markov Decision Process. Unfortunately, in this situation it isn't possible to transform a MMDP to a belief MDP  
         * 
         * @return a belief MDP
         */
        std::shared_ptr<SerializedBeliefMDP<SerializedBeliefState, number, Joint<number>>> toBeliefMDP();

        /**
         * @brief Get the Obs Space of the SerializedMPOMDP. In this situation, it is the same as the ObsSpace of a Dec-Pomdp
         * 
         * @return std::shared_ptr<MultiDiscreteSpace<number>> 
         */
        std::shared_ptr<DiscreteSpace<observation_type>> getObsSpace(number = 0) const;

        /**
         * @brief Get the Obs Space of a precise agent
         * 
         * @param ag_id is the identifiant of a precise agent
         * @return std::shared_ptr<DiscreteSpace<number>> 
         */
        std::vector<observation_type> getObsSpaceAt(number) const;

        std::shared_ptr<MultiDiscreteSpace<number>> getJointActionSpace() const;

        std::shared_ptr<DiscreteSpace<number>> getActionSpace(number t = 0) const;

        //! \fn       double (action, observation, state) const
        //! \param    x a specific current state
        //! \param    u a specific action
        //! \param    z a specific observation
        //! \param    y a specific next state
        //! \brief    Returns probability
        //! \return   value
        double getObservationProbability(const state_type, const action_type, const observation_type, const state_type) const;

        /**
         * @brief Return the current problem
         * 
         * @return std::shared_ptr<SerializedMPOMDP> 
         */
        std::shared_ptr<SerializedMPOMDP> getptr();

        /**
         * @brief Return a observation type associated to a number
         * 
         */
        const observation_type getObservation(number);

        std::vector<std::vector<Matrix>> getDynamics();

        /**
         * @brief Get the Reward for a precise serialized_state and the action of the last agent
         * 
         * @param serialized_state 
         * @param action 
         * @return double 
         */
        double getReward(const SerializedState &, const number &) const;

        /**
         * @brief Get the Reward for a precise serialized_state and the Joint<action>
         * 
         * @param serialized_state 
         * @param action 
         * @return double 
         */
        double getReward(const SerializedState &, const Joint<number> &) const;

        /**
         * @brief The reward of the simultaneous mmdp. 
         * 
         * @return std::shared_ptr<TReward> 
         */
        std::shared_ptr<Reward> getReward() const; // Pour le moment, obligé de crée cette variable

        number joint2single(const Joint<number>);

        void setDynamics();

        double getDynamics(const SerializedState serialized_state, const number action, const Joint<number> joint_obs, const SerializedState serialized_state_next) const;

        double getTransitionProbability(const SerializedState &s, const number &action, const SerializedState &s_) const;

        void setInternalState(SerializedState new_i_state);

        void setPlanningHorizon(number horizon);

    protected:
        Joint<number> empty_serial_observation;

        std::shared_ptr<DiscreteDecPOMDP> decpomdp_;

        std::shared_ptr<MultiSpace<DiscreteSpace<observation_type>>> serialized_observation_space_;

        std::unordered_map<state_type, std::unordered_map<action_type, std::unordered_map<state_type, std::set<observation_type>>>> reachable_obs_state_space;
        std::unordered_map<state_type, std::unordered_map<action_type, std::unordered_map<observation_type, std::unordered_map<state_type, double>>>> dynamics;
        std::unordered_map<state_type, std::unordered_map<action_type, std::unordered_map<observation_type, std::unordered_map<state_type, double>>>> observation_probability;

        // ObservationDynamics obs_dynamics;

        // std::vector<std::vector<MappedMatrix<state_type, state_type>>> dyn_matrix

        /**
         * @brief Initialize Serial Observation Space
         * 
         */
        void setupObsSpace();

        /**
         * @brief Initialize "reachable_observation_space"
         * 
         */

        void setReachableObsSpace();

        void createInitSerializedStateSpace();
        void createInitReachableStateSpace();
    };
} // namespace sdm
#include <sdm/world/serialized_mpomdp.tpp>