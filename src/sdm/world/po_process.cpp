#include <sdm/world/po_process.hpp>

namespace sdm
{

    PartiallyObservableProcess::PartiallyObservableProcess()
    {
    }

    PartiallyObservableProcess::PartiallyObservableProcess(const DiscreteSpace<number> &state_space, const MultiDiscreteSpace<number> &obs_spaces) : StochasticProcess(state_space), obs_spaces_(obs_spaces)
    {
    }

    PartiallyObservableProcess::PartiallyObservableProcess(const DiscreteSpace<number> &state_space, const MultiDiscreteSpace<number> &obs_spaces, const Vector &start_distrib) : StochasticProcess(state_space, start_distrib), obs_spaces_(obs_spaces)
    {
    }

    const MultiDiscreteSpace<number> &PartiallyObservableProcess::getObsSpace() const
    {
        return this->obs_spaces_;
    }

    number PartiallyObservableProcess::getNumJObservations() const
    {
        return this->getObsSpace().getNumJointItems();
    }

    number PartiallyObservableProcess::getNumObservations(number ag_id) const
    {
        return this->getObsSpace().getSpace(ag_id)->getNumItems();
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