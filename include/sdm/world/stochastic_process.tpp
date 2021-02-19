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
    typename StochasticProcess<TStateSpace, TDistrib>::value_type StochasticProcess<TStateSpace, TDistrib>::init()
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
    typename StochasticProcess<TStateSpace, TDistrib>::value_type StochasticProcess<TStateSpace, TDistrib>::getInternalState() const
    {
        return this->internal_state_;
    }

    template <typename TStateSpace, typename TDistrib>
    void StochasticProcess<TStateSpace, TDistrib>::setInternalState(StochasticProcess<TStateSpace, TDistrib>::value_type state)
    {
        this->internal_state_ = state;
    }

    template <typename TStateSpace, typename TDistrib>
    TStateSpace StochasticProcess<TStateSpace, TDistrib>::getStateSpace() const
    {
        return this->state_space_;
    }

    template <typename TInteger>
    DiscreteStochasticProcess<TInteger>::DiscreteStochasticProcess()
    {
    }

    template <typename TInteger>
    DiscreteStochasticProcess<TInteger>::DiscreteStochasticProcess(TInteger num_states)
    {
        std::vector<TInteger> v(num_states);
        std::iota(v.begin(), v.end(), 0);
        this->state_space_ = DiscreteSpace<TInteger>(v);
    }

    template <typename TInteger>
    DiscreteStochasticProcess<TInteger>::DiscreteStochasticProcess(DiscreteSpace<TInteger> state_sp, std::discrete_distribution<TInteger> start_distrib) : StochasticProcess<DiscreteSpace<TInteger>, std::discrete_distribution<TInteger>>(state_sp, start_distrib)
    {
    }

    template <typename TInteger>
    DiscreteStochasticProcess<TInteger>::DiscreteStochasticProcess(std::initializer_list<double> start_distrib) : DiscreteStochasticProcess(start_distrib.size())
    {
        this->start_distrib_ = std::discrete_distribution<TInteger>(start_distrib);
    }

    template <typename TInteger>
    int DiscreteStochasticProcess<TInteger>::getNumStates() const
    {
        return this->getStateSpace().getNumItems();
    }

} // namespace sdm