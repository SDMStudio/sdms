#include <sdm/world/base/po_process_base.hpp>

namespace sdm
{

    template <typename TStateSpace, typename TObsSpace, typename TDistrib>
    PartiallyObservableProcessBase<TStateSpace, TObsSpace, TDistrib>::PartiallyObservableProcessBase()
    {
    }

    template <typename TStateSpace, typename TObsSpace, typename TDistrib>
    PartiallyObservableProcessBase<TStateSpace, TObsSpace, TDistrib>::PartiallyObservableProcessBase(std::shared_ptr<TStateSpace> state_space, std::shared_ptr<TObsSpace> obs_space) : StochasticProcessBase<TStateSpace, TDistrib>(state_space), obs_space_(obs_space)
    {
    }

    template <typename TStateSpace, typename TObsSpace, typename TDistrib>
    PartiallyObservableProcessBase<TStateSpace, TObsSpace, TDistrib>::PartiallyObservableProcessBase(std::shared_ptr<TStateSpace> state_space, std::shared_ptr<TObsSpace> obs_space, TDistrib start_distrib) : StochasticProcessBase<TStateSpace, TDistrib>(state_space, start_distrib), obs_space_(obs_space)
    {
    }

    template <typename TStateSpace, typename TObsSpace, typename TDistrib>
    std::shared_ptr<TObsSpace> PartiallyObservableProcessBase<TStateSpace, TObsSpace, TDistrib>::getObsSpace() const
    {
        return this->obs_space_;
    }

    template <typename TStateSpace, typename TObsSpace, typename TDistrib>
    void PartiallyObservableProcessBase<TStateSpace, TObsSpace, TDistrib>::setObsSpace(std::shared_ptr<TObsSpace> obs_space)
    {
        this->obs_space_ = obs_space;
    }

} // namespace sdm