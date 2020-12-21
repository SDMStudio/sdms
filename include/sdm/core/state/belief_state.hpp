#include <sdm/utils/struct/recursive_map.hpp>
#include <sdm/core/state/history.hpp>

namespace sdm
{
    template <typename TPerceivedState, typename TRealState>
    using BeliefState = RecursiveMap<TPerceivedState, TRealState, double>;

    // template <typename TPerceivedState>
    // using BeliefState = RecursiveMap<TPerceivedState, Vector>;

    using BeliefState = BeliefState<act_obs_history_tree_p<number, number>, number, double>;
} // namespace sdm