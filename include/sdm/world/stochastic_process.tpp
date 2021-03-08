#include <sdm/world/stochastic_process.hpp>
#include <sdm/common.hpp>

namespace sdm
{
    template <typename TStateSpace, typename TDistrib>
    StochasticProcess<TStateSpace, TDistrib>::StochasticProcess()
    {
    }

    template <typename TStateSpace, typename TDistrib>
    StochasticProcess<TStateSpace, TDistrib>::StochasticProcess(TStateSpace state_sp) : state_space_(state_sp)
    {
    }

    template <typename TStateSpace, typename TDistrib>
    StochasticProcess<TStateSpace, TDistrib>::StochasticProcess(TStateSpace state_sp, TDistrib start_distrib) : state_space_(state_sp), start_distrib_(start_distrib)
    {
    }

    template <typename TStateSpace, typename TDistrib>
    typename StochasticProcess<TStateSpace, TDistrib>::state_type StochasticProcess<TStateSpace, TDistrib>::init()
    {
        this->internal_state_ = this->start_distrib_(sdm::common::global_urng());
        return this->internal_state_;
    }

    template <typename TStateSpace, typename TDistrib>
    void StochasticProcess<TStateSpace, TDistrib>::setStartDistrib(TDistrib start_distrib)
    {
        this->start_distrib_ = start_distrib;
    }

    template <typename TStateSpace, typename TDistrib>
    TDistrib StochasticProcess<TStateSpace, TDistrib>::getStartDistrib() const
    {
        return this->start_distrib_;
    }

    template <typename TStateSpace, typename TDistrib>
    typename StochasticProcess<TStateSpace, TDistrib>::state_type StochasticProcess<TStateSpace, TDistrib>::getInternalState() const
    {
        return this->internal_state_;
    }

    template <typename TStateSpace, typename TDistrib>
    void StochasticProcess<TStateSpace, TDistrib>::setInternalState(StochasticProcess<TStateSpace, TDistrib>::state_type state)
    {
        this->internal_state_ = state;
    }

    template <typename TStateSpace, typename TDistrib>
    TStateSpace StochasticProcess<TStateSpace, TDistrib>::getStateSpace() const
    {
        return this->state_space_;
    }

} // namespace sdm