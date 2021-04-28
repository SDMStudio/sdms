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

        std::unordered_map<Joint<number>, std::unordered_map<TState, std::set<Joint<number>>>> reachable_obs_state_space;

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

        const std::set<Joint<number>>& getReachableObservations(const Joint<number>& , const TState& );

        std::shared_ptr<SerializedMMDP<>> toMDP();

        /**
         * @brief Get the corresponding Belief Markov Decision Process. Unfortunately, in this situation it isn't possible to transform a MMDP to a belief MDP  
         * 
         * @return a belief MDP
         */
        std::shared_ptr<BeliefMDP<BeliefState, number, number>> toBeliefMDP(); 

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
    };
} // namespace sdm
#include <sdm/world/serialized_mpomdp.tpp>