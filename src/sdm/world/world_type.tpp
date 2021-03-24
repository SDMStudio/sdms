#include <sdm/world/world_type.hpp>

namespace sdm
{

    template <typename TState, typename TAction>
    struct WorldType
    {
        template <bool is_solving_mdp>
        static std::enable_if_t<is_solving_mdp, DiscreteMDP>
        getAssociatedProblem()
        {
            return DiscreteMDP();
        }

        template <bool is_solving_mmdp>
        static std::enable_if_t<is_solving_mmdp, DiscreteMMDP>
        getAssociatedProblem()
        {
            return DiscreteMMDP();
        }

        template <bool is_solving_pomdp>
        static std::enable_if_t<is_solving_pomdp, DiscretePOMDP>
        getAssociatedProblem()
        {
            return DiscretePOMDP();
        }

        template <bool is_solving_decpomdp>
        static std::enable_if_t<is_solving_decpomdp, DiscreteDecPOMDP>
        getAssociatedProblem()
        {
            return DiscreteDecPOMDP();
        }

        template <bool is_solving_decpomdp>
        static std::enable_if_t<is_solving_decpomdp, DiscreteDecPOMDP>
        getAssociatedProblem()
        {
            return DiscreteDecPOMDP();
        }
    };
} // namespace sdm