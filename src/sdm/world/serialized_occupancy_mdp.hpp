#pragma once

#include <sdm/types.hpp>

#include <sdm/core/state/serialized_state.hpp>
#include <sdm/core/state/serialized_occupancy_state.hpp>
#include <sdm/core/action/det_decision_rule.hpp>

#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/world/serialized_mpomdp.hpp>
#include <sdm/world/base/base_serial_occupancy_mdp.hpp>

#include <sdm/utils/linear_algebra/vector.hpp>

namespace sdm
{
    /**
     * @brief An Serialized occupancy MDP is a subclass of continuous state MDP where states are seriliazed occupancy states and the resolution is serialized. 
     * In the general case, a Serialized occupancy state refers to the knowledge that a central planner can have access to take decisions at a precise agent. 
     * But in this implementation we call serialized occupancy state a distribution over serialized state and joint histories .
     * 
     * @tparam TState refers to an serialized occupancy state type 
     * @tparam TAction refers to a occupancy action type 
     */
    template <typename TState = SerializedOccupancyState<SerializedState, JointHistoryTree_p<number>>, typename TAction = DeterministicDecisionRule<HistoryTree_p<number>, number>>
    class SerializedOccupancyMDP : public BaseSerializedOccupancyMDP<TState, TAction>
    {
    protected : 

        void initialize(number);

    public:
        using state_type = TState;
        using action_type = TAction;

        SerializedOccupancyMDP();
        SerializedOccupancyMDP(std::string, number = -1);
        SerializedOccupancyMDP(std::shared_ptr<DiscreteDecPOMDP>, number = -1);

        /**
         * @brief Get the actions availables at a specific state
         * 
         * @param state the state
         * @return the action space 
         */
        // std::shared_ptr<DiscreteSpace<TAction>> getActionSpaceAt(const TState &);
        
        TState nextState(const TState &, const TAction &, number, std::shared_ptr<HSVI<TState, TAction>>, bool) const;
        TState nextState(const TState &, const TAction &, number = 0, std::shared_ptr<HSVI<TState, TAction>> = nullptr) const;


        double getReward(const TState &, const TAction &) const;

        TState nextStateSerialStep(const TState &ostate, const TAction &indiv_dr) const;
    };

    // ##############################################################################################################################
    // ############################ SPECIALISATION FOR BeliefOccupancyState Structure ###############################################
    // ##############################################################################################################################

    // /**
    //  * @brief Specialisation of occupancy mdp in the case of occupancy states. 
    //  * 
    //  * @tparam TActionDescriptor the action type
    //  * @tparam TObservation the observation type
    //  * @tparam TActionPrescriptor the action type (controller's one)
    //  */
    // template <typename TActionDescriptor, typename TObservation, typename TActionPrescriptor>
    // class SerializedOccupancyMDP<SerializedOccupancyState<BeliefStateGraph_p<TActionDescriptor, TObservation>, JointHistoryTree_p<TObservation>>, TActionPrescriptor>
    //     : public BaseSerializedOccupancyMDP<SerializedOccupancyState<BeliefStateGraph_p<TActionDescriptor, TObservation>, JointHistoryTree_p<TObservation>>, TActionPrescriptor>
    // {
    // public:
    //     using state_type = SerializedOccupancyState<BeliefStateGraph_p<TActionDescriptor, TObservation>, JointHistoryTree_p<TObservation>>;
    //     using action_type = TActionPrescriptor;
    //     using observation_type = TObservation;

    //     SerializedOccupancyMDP();
    //     SerializedOccupancyMDP(std::string dpomdp_name, number max_history_length = -1);
    //     SerializedOccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> dpomdp, number max_history_length = -1);

    //     void initialize(number history_length);

    //     state_type nextState(const state_type &, const action_type &, number, std::shared_ptr<HSVI<state_type, action_type>>, bool) const;
    //     state_type nextState(const state_type &, const action_type &, number = 0, std::shared_ptr<HSVI<state_type, action_type>> = nullptr) const;
    //     double getReward(const state_type &occupancy_state, const action_type &decision_rule) const;
    // };


} // namespace sdm
#include <sdm/world/serialized_occupancy_mdp.tpp>
