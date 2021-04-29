#pragma once

#include <sdm/types.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/world/belief_mdp.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/state/serialized_state.hpp>

#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/core/action/det_decision_rule.hpp>

#include <sdm/world/serialized_mmdp_structure.hpp>

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
    class SerializedMMDP : public SolvableByHSVI<TState, TAction>,
                           public std::enable_shared_from_this<SerializedMMDP<TState, TAction>>
    {
    public:
        using state_type = TState;
        using action_type = TAction;

        SerializedMMDP();
        SerializedMMDP(std::shared_ptr<DiscreteMMDP>);
        SerializedMMDP(std::string);

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
        SerializedMMDPStructure<TState, TAction>* getUnderlyingProblem();

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
        TState nextState(const TState&, const TAction&, number = 0, std::shared_ptr<HSVI<TState, TAction>> = nullptr) const;

        std::shared_ptr<DiscreteSpace<TAction>> getActionSpaceAt(const TState &serialized_state) ;        

        /**
         * @brief Get the Expected Next Value object
         * 
         * @return double 
         */
        double getExpectedNextValue(std::shared_ptr<ValueFunction<TState, TAction>>, const TState&, const TAction&, number = 0) const;

        /**
         * @brief Get the Reward for a precise serialized_state and the action of the last agent
         * 
         * @param serialized_state 
         * @param action 
         * @return double 
         */
        double getReward(const TState &,const TAction &) const;

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

  
    protected:
        /**
         * @brief the simultaneous multi-agent Markov decision process
         */
        std::shared_ptr<SerializedMMDPStructure<TState,number>> serial_mmdp_;

    };
} // namespace sdm
#include <sdm/world/serialized_mmdp.tpp>
