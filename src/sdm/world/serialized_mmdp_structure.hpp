#pragma once

#include <sdm/types.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/world/belief_mdp.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/state/serialized_state.hpp>

#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/core/action/det_decision_rule.hpp>
//#include <sdm/utils/decision_rules/det_decision_rule.hpp>

namespace sdm
{
    /**
     * @brief An Serialized MDP is a subclass of MDP where states are serialized states. 
     * In the general case, a serialized state refers to the whole knowledge that a central planner can have access to take decisions at the time step of an precise agent. 
     * 
     * @tparam TState refers to the serialized state type
     * @tparam TAction refers to the number type
     */
    class SerializedMMDPStructure : public DecisionProcessBase<MultiSpace<DiscreteSpace<SerializedState>>, MultiDiscreteSpace<number>, std::discrete_distribution<number>>,
                           public std::enable_shared_from_this<SerializedMMDPStructure>
    {
    public:
        using state_type = SerializedState;
        using action_type = number;

        SerializedMMDPStructure();
        SerializedMMDPStructure(std::shared_ptr<DiscreteMMDP>);
        SerializedMMDPStructure(std::string);

        /**
         * @brief Return the Serialized Discount
         * 
         * @param t 
         * @return double 
         */
        double getDiscount(number = 0) const;

        /**
         * @brief Get the Next State Space object for a precise serialized state
         * @comment: surcharger la fonction 'getReachableSerialStates(...)'
         * @param serialized_state 
         * @return std::shared_ptr<DiscreteSpace<SerializedState>> 
         */
        const std::set<state_type>& getReachableSerialStates(const state_type&, const action_type&) const;

        /**
         * @brief Get the Reward for a precise serialized_state and the action of the last agent
         * 
         * @param serialized_state 
         * @param action 
         * @return double 
         */
        double getReward(const state_type &,const action_type &) const;

        /**
         * @brief Get the Reward for a precise serialized_state and the Joint<action>
         * 
         * @param serialized_state 
         * @param action 
         * @return double 
         */
        double getReward(const state_type &,const Joint<action_type> &) const;

        /**
         * @brief The reward of the simultaneous mmdp. 
         * 
         * @return std::shared_ptr<TReward> 
         */
        std::shared_ptr<Reward> getReward() const; // Pour le moment, obligé de crée cette variable

        /**
         * @brief Get the probability to be in serialized_state_next giving a precise serialized_state and the action of the last agent. 
         * @param serialized_state 
         * @param action 
         * @param serialized_state_next 
         * @return double 
         */
        double getTransitionProbability(const state_type &,const action_type &, const state_type &) const;
        
        void setInternalState(state_type );
        
        void setPlanningHorizon(number );      

        /**
         * @brief Return the current problem
         * 
         * @return std::shared_ptr<SerializedMMDPStructure> 
         */
        std::shared_ptr<SerializedMMDPStructure> getptr();

        /**
         * @brief Transform the current problem to a MDP. In this situation, it will returr the current problem
         * 
         * @return std::shared_ptr<SerializedMMDPStructure<TState, TAction>> 
         */
        std::shared_ptr<SerializedMMDPStructure> toMDP();

        /**
         * @brief Get the corresponding Belief Markov Decision Process. Unfortunately, in this situation it isn't possible to transform a Serialized MMDP to a belief MDP  
         * 
         * @return a belief MDP
         */
        std::shared_ptr<BeliefMDP<BeliefState, number, number>> toBeliefMDP(); 

        /**
         * @brief Return the number of agent
         * 
         * @return number 
         */
        number getNumAgents() const;

        /**
         * \brief Getter for the serial action space
         */
        std::shared_ptr<DiscreteSpace<action_type>> getActionSpace(number = 0) const;

        /**
         * \brief Getter for the Joint action space
         */
        std::shared_ptr<MultiDiscreteSpace<action_type>> getJointActionSpace() const;

        /**
         * \brief Getter for the serial action space
         */
        std::shared_ptr<DiscreteSpace<state_type>> getStateSpace(number = 0) const;
  
    protected:
        /**
         * @brief the simultaneous multi-agent Markov decision process
         */
        std::shared_ptr<DiscreteMMDP> mmdp_;

        /**
         * @brief Refer to the Serialized State Space
         * 
         */
        std::shared_ptr<MultiSpace<DiscreteSpace<state_type>>> serialized_state_space_;

        /**
         * @brief Map (serial state, seial action) to Set of reachable seial states
         */
        std::unordered_map<state_type, std::unordered_map<action_type, std::set<state_type>>> reachable_state_space;

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

    };
} // namespace sdm
#include <sdm/world/serialized_mmdp_structure.tpp>
