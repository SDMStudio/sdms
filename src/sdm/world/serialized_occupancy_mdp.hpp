#pragma once

#include <sdm/types.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/world/discrete_mdp.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/serialized_mpomdp.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/state/serialized_occupancy_state.hpp>
#include <sdm/core/state/serialized_state.hpp>

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
    class SerializedOccupancyMDP : public SolvableByHSVI<TState, TAction>
    {
    protected:
        std::shared_ptr<SerializedMPOMDP> serialized_mpomdp_;
        TState istate_, cstate_;

    public:
        using state_type = TState;
        using action_type = TAction;

        SerializedOccupancyMDP();
        SerializedOccupancyMDP(std::string, number = -1);
        SerializedOccupancyMDP(std::shared_ptr<DiscreteDecPOMDP>, number = -1);

        bool isSerialized() const;
        std::shared_ptr<DiscreteSpace<TAction>> getActionSpaceAt(const TState &);

        SerializedMPOMDP *getUnderlyingProblem();

        TState getInitialState();
        TState nextState(const TState &, const TAction &, number = 0, std::shared_ptr<HSVI<TState, TAction>> = nullptr) const;

        double getReward(const TState &, const TAction &) const;
        double getExpectedNextValue(std::shared_ptr<ValueFunction<TState, TAction>>, const TState &, const TAction &, number = 0) const;

        std::shared_ptr<SerializedMMDP> toMDP();

        /**
         * @brief Get the corresponding Belief Markov Decision Process. It corresponds to the reformulation of the original POMP in a MDP where the state space is the space of beliefs. 
         * 
         * @return a belief MDP
         */
        std::shared_ptr<BeliefMDP<BeliefState, number, number>> toBeliefMDP()
        {
            sdm::exception::NotImplementedException();
        }
    };
} // namespace sdm
#include <sdm/world/serialized_occupancy_mdp.tpp>
