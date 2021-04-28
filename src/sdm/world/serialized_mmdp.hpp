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

/*
De : Jilles 
A : Qui de droit

    Désolé mais ce code est illisible -- il n'y a aucun commentaire. 
    Gardez à l'esprit que SDMS va être diffusé à la communauté. 
    Si moi, j'ai du mal beaucoup n'y arriveront pas commentez vos codes SVP. 

    Je vais etre sincère, ce code est davantage un brouillon qu'autre chose. 
    Il est à corrigé très rapidement.
*/


namespace sdm
{
    /**
     * @brief An Serialized MDP is a subclass of MDP where states are serialized states. 
     * In the general case, a serialized state refers to the whole knowledge that a central planner can have access to take decisions at the time step of an precise agent. 
     * 
     * @tparam TState refers to the serialized state type
     * @tparam TAction refers to the number type
     */
    template <typename TState = SerializedState, typename TAction = number>
    class SerializedMMDP : public DecisionProcessBase<MultiSpace<DiscreteSpace<TState>>, MultiDiscreteSpace<TAction>, std::discrete_distribution<number>>,
                           //public SolvableByHSVI<TState, TAction>,
                           public std::enable_shared_from_this<SerializedMMDP<TState, TAction>>
    {
    public:
        using state_type = TState;
        using action_type = TAction;

        SerializedMMDP();
        SerializedMMDP(std::shared_ptr<DiscreteMMDP>);
        SerializedMMDP(std::string);


        /**
         * @brief Return the Serialized Discount
         * 
         * @param t 
         * @return double 
         */
        double getDiscount(number = 0) const;

        /**
         * @brief 
         *  
         * @return true 
         * @return false 
         */
        bool isSerialized() const;

        /**
         * @brief Get itself ...
         * 
         * @return SerializedMMDP<TState,TAction>* 
         */
        SerializedMMDP<TState, TAction>* getUnderlyingProblem();

        /**
         * @brief Get the initial serial state 
         * @return TState 
         */
        TState getInitialState();

        /**
         * @brief 
         * 
         * @return TState 
         */
        TState nextState(const TState&, const TAction&, number = 0, HSVI<TState, TAction>* = nullptr) const;
        

        /**
         * @brief Get the Expected Next Value object
         * 
         * @return double 
         */
        double getExpectedNextValue(ValueFunction<TState, TAction>*, const TState&, const TAction&, number = 0) const;

        /**
         * @brief Get the Next State Space object for a precise serialized state
         * @comment: surcharger la fonction 'getReachableSerialStates(...)'
         * @param serialized_state 
         * @return std::shared_ptr<DiscreteSpace<SerializedState>> 
         */
        const std::set<TState>& getReachableSerialStates(const TState&, const TAction&) const;

        /**
         * @brief Get All the Serialized State Space
         * 
         * @return std::shared_ptr<MultiSpace<DiscreteSpace<SerializedState>>> 
         */
        std::shared_ptr<MultiSpace<DiscreteSpace<SerializedState>>> getSerialStateSpace() const;


        /**
         * @brief Get the Action Space object
         * 
         * @return std::shared_ptr<MultiDiscreteSpace<TAction>> 
         */
        std::shared_ptr<MultiDiscreteSpace<TAction>> getSerialActionSpace(number )const;

        /**
         * @brief Get the Reward for a precise serialized_state and the action of the last agent
         * 
         * @param serialized_state 
         * @param action 
         * @return double 
         */
        double getReward(const TState &,const TAction &) const;

        /**
         * @brief Get the probability to be in serialized_state_next giving a precise serialized_state and the action of the last agent. 
         * @param serialized_state 
         * @param action 
         * @param serialized_state_next 
         * @return double 
         */
        double getTransitionProbability(const TState &,const TAction &, const TState &) const;

        TState getInternalState() const;
        
        void setInternalState(TState );
        
        void setPlanningHorizon(number );      

        /**
         * @brief Return the current problem
         * 
         * @return std::shared_ptr<SerializedMMDP> 
         */
        std::shared_ptr<SerializedMMDP> getptr();

        /**
         * @brief Transform the current problem to a MDP. In this situation, it will returr the current problem
         * 
         * @return std::shared_ptr<SerializedMMDP<TState, TAction>> 
         */
        std::shared_ptr<SerializedMMDP<TState, TAction>> toMDP();

        /**
         * @brief Get the corresponding Belief Markov Decision Process. Unfortunately, in this situation it isn't possible to transform a Serialized MMDP to a belief MDP  
         * 
         * @return a belief MDP
         */
        std::shared_ptr<BeliefMDP<BeliefState, number, number>> toBeliefMDP(); 

        number getNumAgents() const;
  
    protected:
        /**
         * @brief the simultaneous multi-agent Markov decision process
         */
        std::shared_ptr<DiscreteMMDP> mmdp_;

        /**
         * @brief Refer to the Serialized State Space
         * 
         */
        std::shared_ptr<MultiSpace<DiscreteSpace<SerializedState>>> serialized_state_space_;

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
#include <sdm/world/serialized_mmdp.tpp>
