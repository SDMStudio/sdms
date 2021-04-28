#pragma once

#include <sdm/types.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/world/belief_mdp.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/state/serialized_state.hpp>

#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/core/action/det_decision_rule.hpp>

#include <sdm/world/serialized_mmdp.hpp>

namespace sdm
{
    /**
     * @brief An Serialized MDP is a subclass of MDP where states are serialized states. 
     * In the general case, a serialized state refers to the whole knowledge that a central planner can have access to take decisions at the time step of an precise agent. 
     * 
     * @tparam TState refer to the serialized state type
     * @tparam TAction refer to the number type
     */
    template <typename TState = SerializedState, typename TAction = number>
    class SerializedMPOMDP : public SerializedMMDP<SerializedState,number>
    {
    protected:
        std::shared_ptr<DiscreteDecPOMDP> decpomdp_;
        std::shared_ptr<MultiDiscreteSpace<number>> serialized_observation_space_;
        std::shared_ptr<MultiSpace<DiscreteSpace<SerializedState>>> serialized_state_space_;

        std::unordered_map<TAction, std::unordered_map<TState, std::set<number>>> reachable_obs_state_space;

        /**
         * @brief Initialize Serial Observation Space
         * 
         */
        void createInitSerialObservationSpace();

        /**
         * @brief Initialize "reachable_obs_state_space"
         * 
         */
        void createInitReachableObsStateSpace();

    public:
        using state_type = TState;
        using action_type = TAction;

        SerializedMPOMDP();
        SerializedMPOMDP(std::shared_ptr<DiscreteDecPOMDP>);
        SerializedMPOMDP(std::string);

        const std::set<number>& getReacheableObservations(const TAction& , const TState& );

    //     bool isSerialized() const;

    //     SerializedMPOMDP<TState, TAction> *getUnderlyingProblem();

    //     TState getInitialState();
        
    //     //TState nextState(const TState &Serial_Occupancy_state, const TAction &Serial_Occupancy_action, int t = 0, HSVI<TState, TAction> *hsvi = nullptr) const;

    //     //double getExpectedNextValue(ValueFunction<TState, TAction> *value_function, const TState &Serial_Occupancy_state, const TAction &Serial_Occupancy_action, int t = 0) const;

        std::shared_ptr<SerializedMMDP<>> toMDP();

        /**
         * @brief Get the corresponding Belief Markov Decision Process. Unfortunately, in this situation it isn't possible to transform a MMDP to a belief MDP  
         * 
         * @return a belief MDP
         */
        std::shared_ptr<BeliefMDP<BeliefState, number, number>> toBeliefMDP(); 


        // /**
        //  * @brief For a precise SerializedState, this function return the next SerializedState possible
        //  * 
        //  * @param state 
        //  * @return std::shared_ptr<DiscreteSpace<typename TState>> 
        //  */
        // std::shared_ptr<DiscreteSpace<typename TState>> getNextSerializedStateSpace(const typename TState &state) const;

        // /**
        //  * @brief For a precise SerializedState, this function return the next SerializedState possible
        //  * 
        //  * @param state is a serialized state
        //  * @param action is an action
        //  * @return std::shared_ptr<DiscreteSpace<typename TState>> 
        //  */
        // std::shared_ptr<DiscreteSpace<typename TState>> getNextSerializedStateSpace(const typename TState &state, const number &action) const;

        // /**
        //  * @brief THis function return all the serializedState possible
        //  * 
        //  * @return std::shared_ptr<MultiSpace<DiscreteSpace<typename TState>>> 
        //  */
        // std::shared_ptr<MultiSpace<DiscreteSpace<typename TState>>> getSerializedStateSpace() const; 

        // /**
        //  * @brief Get the Serialized State Space of a precise agent
        //  * 
        //  * @param ag_id is the identifiant of an agent 
        //  * @return std::shared_ptr<DiscreteSpace<typename TState>> 
        //  */
        // std::shared_ptr<DiscreteSpace<typename TState>> getSerializedStateSpaceAt(number ag_id) const;

        // std::shared_ptr<MultiSpace<DiscreteSpace<typename TState>>> getStateSpace() const; 
        // std::shared_ptr<DiscreteSpace<typename TState>> getStateSpaceAt(number ag_id) const;

        /**
         * @brief Get the Obs Space of the SerializedMPOMDP. In this situation, it is the same as the ObsSpace of a Dec-Pomdp
         * 
         * @return std::shared_ptr<MultiDiscreteSpace<number>> 
         */
        std::shared_ptr<MultiDiscreteSpace<number>> getObsSpace() const;

        /**
         * @brief Get the Obs Space of a precise agent
         * 
         * @param ag_id is the identifiant of a precise agent
         * @return std::shared_ptr<DiscreteSpace<number>> 
         */
        std::shared_ptr<DiscreteSpace<number>> getObsSpaceAt(number ) const;

        double getObservationProbability(const TAction &, const Joint<number> ,const TState &) const;

        double getDynamics(const TState &,const TAction ,const Joint<number> ,const TState &) const;

        //std::shared_ptr<DiscreteSpace<SerializedState>> getReachableSerialStates(const TState&, const TAction&) const;


        // /**
        //  * @brief Get the Obs Dynamics 
        //  * 
        //  * @param s is a hidden state
        //  * @param joint_action is a specific joint action
        //  * @param joint_obs  is a specific joint observation
        //  * @param s_ is a hidden state
        //  * @return double 
        //  */
        // double getObsDynamics(const number &s,const number joint_action,const number joint_obs,const number &s_) const;

        // /**
        //  * @brief Get the Obs Dynamics. This function is more general because it will do multiple verification 
        //  * 
        //  * @param s is a serialized State
        //  * @param action is a private action 
        //  * @param joint_obs is a vector of observation
        //  * @param s_ is a serialized State
        //  * @return double 
        //  */
        // double getObsDynamics(const typename TState &s,const number action,const Joint<number> joint_obs,const typename TState &s_) const;

        // /**
        //  * @brief Get the Action Space of a SerializedMPOMDP. In this situation, it is the same as the ActionSpace of a Dec-Pomdp
        //  * 
        //  * @return std::shared_ptr<MultiDiscreteSpace<number>> 
        //  */
        // std::shared_ptr<MultiDiscreteSpace<number>> getActionSpace()const;
        
        // /**
        //  * @brief Get the Action Space of a precise agent. In this situation, it is the same as the ActionSpaceAt of a Dec-Pomdp
        //  * 
        //  * @param state 
        // //  * @return std::shared_ptr<DiscreteSpace<number>> 
        // //  */
        // // std::shared_ptr<DiscreteSpace<number>> getActionSpaceAt(number ag_id) ;


        // double getReward(const typename TState &s,const number &action) const;
        // double getReward(const typename TState &s,const Joint<number> &action) const;


        // std::shared_ptr<Reward> getReward() const; // A modifier, car pour 

        // double getDynamics(const typename TState &s,const number &action, const typename TState &s_) const;

        // number getNumAgents() const;

        // std::discrete_distribution<number> getStartDistrib() const;

        // void setPlanningHorizon(number horizon);

        // //TState getInternalState() const;
        // void setInternalState(const typename TState  new_i_state);
        
    };
} // namespace sdm
#include <sdm/world/serialized_mpomdp.tpp>