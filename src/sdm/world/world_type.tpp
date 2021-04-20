#include <sdm/world/world_type.hpp>

namespace sdm
{

    template <typename TState, typename TAction>
    struct WorldType
    {
        template <bool is_mdp>
        static std::enable_if_t<is_mdp, DiscreteMDP>
        getWorldType()
        {
            return DiscreteMDP();
        }

        template <bool is_belief_mdp>
        static std::enable_if_t<is_belief_mdp, BeliefMDP<>>
        getWorldType()
        {
            return BeliefMDP<>();
        }

        template <bool is_serialized_belief_mdp>
        static std::enable_if_t<is_serialized_belief_mdp, SerializedBeliefMDP<>>
        getWorldType()
        {
            return SerializedBeliefMDP<>();
        }

        template <bool is_serialized_mdp>
        static std::enable_if_t<is_serialized_mdp, SerializedMDP<SerializedState, number>>
        getWorldType()
        {
            return SerializedMDP<SerializedState, number>();
        }

        template <bool is_serialized_occupancy_mdp>
        static std::enable_if_t<is_serialized_occupancy_mdp, SerializedOccupancyMDP<>>
        getWorldType()
        {
            return SerializedOccupancyMDP<>();
        }

        template <bool is_occupancy_mdp>
        static std::enable_if_t<is_occupancy_mdp, OccupancyMDP<>>
        getWorldType()
        {
            return OccupancyMDP<>();
        }

        template <bool is_multi_agent>
        static std::enable_if_t<is_multi_agent, DiscreteDecPOMDP>
        getWorldType()
        {
            throw sdm::exception::Exception("Joint<number> cannot suffice to determine SolvableByHSVI problem.");
        }
    };
} // namespace sdm