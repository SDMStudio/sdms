#include <sdm/world/base/stochastic_process_base.hpp>

namespace sdm
{
    template <typename TStateSpace, typename TDistrib>
    StochasticProcessBase<TStateSpace, TDistrib>::StochasticProcessBase()
    {
    }

    template <typename TStateSpace, typename TDistrib>
    StochasticProcessBase<TStateSpace, TDistrib>::StochasticProcessBase(std::shared_ptr<TStateSpace> state_sp) : state_space_(state_sp)
    {
    }

    template <typename TStateSpace, typename TDistrib>
    StochasticProcessBase<TStateSpace, TDistrib>::StochasticProcessBase(std::shared_ptr<TStateSpace> state_sp, TDistrib start_distrib) : state_space_(state_sp), start_distrib_(start_distrib)
    {
    }

    template <typename TStateSpace, typename TDistrib>
    TDistrib StochasticProcessBase<TStateSpace, TDistrib>::getStartDistrib() const
    {
        return this->start_distrib_;
    }

    template <typename TStateSpace, typename TDistrib>
    void StochasticProcessBase<TStateSpace, TDistrib>::setStartDistrib(TDistrib start_distrib)
    {
        this->start_distrib_ = start_distrib;
    }

    template <typename TStateSpace, typename TDistrib>
    typename StochasticProcessBase<TStateSpace, TDistrib>::state_type StochasticProcessBase<TStateSpace, TDistrib>::getInternalState() const
    {
        return this->internal_state_;
    }

    template <typename TStateSpace, typename TDistrib>
    void StochasticProcessBase<TStateSpace, TDistrib>::setInternalState(StochasticProcessBase<TStateSpace, TDistrib>::state_type state)
    {
        this->internal_state_ = state;
    }

    template <typename TStateSpace, typename TDistrib>
    std::shared_ptr<TStateSpace> StochasticProcessBase<TStateSpace, TDistrib>::getStateSpace(number) const
    {
        return this->state_space_;
    }

    template <typename TStateSpace, typename TDistrib>
    void StochasticProcessBase<TStateSpace, TDistrib>::setStateSpace(std::shared_ptr<TStateSpace> state_sp)
    {
        this->state_space_ = state_sp;
    }

} // namespace sdm