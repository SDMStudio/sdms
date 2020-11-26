#include <sdm/world/po_process.hpp>

namespace sdm
{

    PartiallyObservableProcess::PartiallyObservableProcess()
    {
    }

    PartiallyObservableProcess::PartiallyObservableProcess(number num_states, std::vector<number> num_obs) : StochasticProcess(num_states), obs_spaces_(num_obs)
    {
    }

    PartiallyObservableProcess::PartiallyObservableProcess(number num_states, std::vector<number> num_obs, const Vector &start_distrib) : StochasticProcess(num_states, start_distrib), obs_spaces_(num_obs)
    {
    }

    PartiallyObservableProcess::PartiallyObservableProcess(const DiscreteSpace &state_space, const MultiDiscreteSpace &obs_spaces) : StochasticProcess(state_space), obs_spaces_(obs_spaces)
    {
    }

    PartiallyObservableProcess::PartiallyObservableProcess(const DiscreteSpace &state_space, const MultiDiscreteSpace &obs_spaces, const Vector &start_distrib) : StochasticProcess(state_space, start_distrib), obs_spaces_(obs_spaces)
    {
    }

    const MultiDiscreteSpace &PartiallyObservableProcess::getObsSpace() const
    {
        return this->obs_spaces_;
    }

    const DiscreteSpace &PartiallyObservableProcess::getObsSpace(number ag_id) const
    {
        return this->getObsSpace().getSpace(ag_id);
    }

    number PartiallyObservableProcess::getNumJObservations() const
    {
        return this->getObsSpace().getNumJElements();
    }

    number PartiallyObservableProcess::getNumObservations(number ag_id) const
    {
        return this->getObsSpace().getNumElements(ag_id);
    }

    std::vector<number> PartiallyObservableProcess::getNumObservations() const
    {
        std::vector<number> v;
        for (number i = 0; i < this->getObsSpace().getNumSpaces(); i++)
        {
            v.push_back(this->getNumObservations(i));
        }
        return v;
    }

} // namespace sdm