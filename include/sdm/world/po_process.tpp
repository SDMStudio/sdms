#include <sdm/world/po_process.hpp>

namespace sdm
{

    template <typename TStateSpace, typename TObsSpace, typename TDistrib>
    PartiallyObservableProcess<TStateSpace, TObsSpace, TDistrib>::PartiallyObservableProcess()
    {
    }

    template <typename TStateSpace, typename TObsSpace, typename TDistrib>
    PartiallyObservableProcess<TStateSpace, TObsSpace, TDistrib>::PartiallyObservableProcess(TStateSpace state_space, TObsSpace obs_spaces) : StochasticProcess<TStateSpace, TDistrib>(state_space), obs_spaces_(obs_spaces)
    {
    }

    template <typename TStateSpace, typename TObsSpace, typename TDistrib>
    PartiallyObservableProcess<TStateSpace, TObsSpace, TDistrib>::PartiallyObservableProcess(TStateSpace state_space, TObsSpace obs_spaces, TDistrib start_distrib) : StochasticProcess<TStateSpace, TDistrib>(state_space, start_distrib), obs_spaces_(obs_spaces)
    {
    }

    template <typename TStateSpace, typename TObsSpace, typename TDistrib>
    TObsSpace PartiallyObservableProcess<TStateSpace, TObsSpace, TDistrib>::getObsSpace() const
    {
        return this->obs_spaces_;
    }

    template <typename TStateSpace, typename TObsSpace, typename TDistrib>
    typename PartiallyObservableProcess<TStateSpace, TObsSpace, TDistrib>::observation_type PartiallyObservableProcess<TStateSpace, TObsSpace, TDistrib>::getObservation() const
    {
        
    }

} // namespace sdm