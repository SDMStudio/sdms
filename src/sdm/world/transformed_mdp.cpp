#include <sdm/world/transformed_mdp.hpp>

namespace sdm
{
    template <typename TState, typename TAction, typename TObserv>
    std::shared_ptr<POSG> TransformedMDP<TState, TAction, TObserv>::getUnderlyingProblem() const
    {
        return this->underlying_problem;
    }

} // namespace sdm
