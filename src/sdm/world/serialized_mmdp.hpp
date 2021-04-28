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
     * @tparam TState refer to the serialized state type
     * @tparam oAction refer to the number type
     */
    template <typename TState = SerializedState,
              typename oAction = number>
    class SerializedMMDP : public SolvableByHSVI<TState, oAction>,
                           public std::enable_shared_from_this<SerializedMMDP<TState, oAction>>,
                           public DecisionProcessBase<MultiSpace<DiscreteSpace<TState>>, MultiDiscreteSpace<oAction>, std::discrete_distribution<number>>

    {
    protected:
        std::shared_ptr<DiscreteMMDP> mmdp_;

        /**
         * @brief Refer to the Serialized State Space
         * 
         */
        std::shared_ptr<MultiSpace<DiscreteSpace<SerializedState>>> serialized_state_space_;

        /**
         * @brief Initialize the attribut "serialized_state_space";
         * 
         */
        void createInitSerializedStateSpace(std::shared_ptr<DiscreteMMDP>);

    public:
        using state_type = TState;
        using action_type = oAction;

        SerializedMMDP(std::shared_ptr<DiscreteMMDP> );
        SerializedMMDP(std::string );


        /**
         * @brief Return the Serialized Discount
         * 
         * @param t 
         * @return double 
         */
        double getDiscount(number =0) const;

        // /**
        //  * @brief Get the Discount object
        //  * 
        //  * @return double 
        //  */
        // double getDiscount() const {
        //     return this->mmdp_->discount_;
        // }

        /**
         * @brief Set the Discount object
         * 
         * @param double : discount 
         */
        void setDiscount(double );

        bool isSerialized() const;
        SerializedMMDP<TState,oAction> *getUnderlyingProblem();

        TState getInitialState();
        TState nextState(const TState&, const oAction &, number = 0, HSVI<TState, oAction> * = nullptr) const;
        double getExpectedNextValue(ValueFunction<TState, oAction> *, const TState &, const oAction &, number = 0) const;


        /**
         * @brief Get the Hidden State Space object
         * 
         * @return std::shared_ptr<DiscreteSpace<number>> 
         */
        std::shared_ptr<DiscreteSpace<number>> getHiddenStateSpace() const;

        /**
         * @brief Get the Next State Space object for a precise serialized state
         * 
         * @param serialized_state 
         * @return std::shared_ptr<DiscreteSpace<SerializedState>> 
         */
        std::shared_ptr<DiscreteSpace<SerializedState>> getNextStateSpace(const TState &) const;

        /**
         * @brief Get All the Serialized State Space
         * 
         * @return std::shared_ptr<MultiSpace<DiscreteSpace<SerializedState>>> 
         */
        std::shared_ptr<MultiSpace<DiscreteSpace<SerializedState>>> getStateSpace() const;

        /**
         * @brief Get the Serialized State Space for a precise agent
         * 
         * @param ag_id 
         * @return std::shared_ptr<DiscreteSpace<SerializedState>> 
         */
        std::shared_ptr<DiscreteSpace<SerializedState>> getStateSpaceAt(number ) const;


        /**
         * @brief Get the Observation Space object. In this situation, it will return the Serialized State Space
         * 
         * @return std::shared_ptr<MultiSpace<DiscreteSpace<SerializedState>>> 
         */
        std::shared_ptr<MultiSpace<DiscreteSpace<SerializedState>>> getObsSpace() const;

        /**
         * @brief Get the Observation Space object of a precise agent. In this situation, it will return the Serialized State Space of a precise agent
         * 
         * @param ag_id 
         * @return std::shared_ptr<DiscreteSpace<SerializedState>> 
         */
        std::shared_ptr<DiscreteSpace<SerializedState>> getObsSpaceAt(number ) const;


        /**
         * @brief Get the Action Space object
         * 
         * @return std::shared_ptr<MultiDiscreteSpace<oAction>> 
         */
        std::shared_ptr<MultiDiscreteSpace<oAction>> getActionSpace()const;

        /**
         * @brief Get the Action Space for a precise serialized_state
         * 
         * @param serialized_state 
         * @return std::shared_ptr<DiscreteSpace<oAction>> 
         */
        std::shared_ptr<DiscreteSpace<oAction>> getActionSpaceAt(const TState &) ;

        /**
         * @brief Get the Reward for a precise serialized_state and the action of the last agent
         * 
         * @param serialized_state 
         * @param action 
         * @return double 
         */
        double getReward(const TState &,const oAction &) const;

        /**
         * @brief Get the Reward for a precise serialized_state and the joint action.
         * 
         * @param serialized_state 
         * @param joint_action 
         * @return double 
         */
        double getReward(const TState &,const Joint<oAction> &) const;


        std::shared_ptr<Reward> getReward() const; 

        /**
         * @brief Get the probability to be in serialized_state_next giving a precise serialized_state and the action of the last agent. 
         * @param serialized_state 
         * @param action 
         * @param serialized_state_next 
         * @return double 
         */
        double getDynamics(const TState &,const oAction &, const TState &) const;

        number getNumAgents() const;

        //TState getInternalState() const;
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
         * @return std::shared_ptr<SerializedMMDP<TState, oAction>> 
         */
        std::shared_ptr<SerializedMMDP<TState, oAction>> toMDP();

        /**
         * @brief Get the corresponding Belief Markov Decision Process. Unfortunately, in this situation it isn't possible to transform a Serialized MMDP to a belief MDP  
         * 
         * @return a belief MDP
         */
        std::shared_ptr<BeliefMDP<BeliefState, number, number>> toBeliefMDP(); 
  
    };
} // namespace sdm
#include <sdm/world/serialized_mmdp.tpp>
