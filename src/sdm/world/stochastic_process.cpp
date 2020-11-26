#include <sdm/world/stochastic_process.hpp>
#include <sdm/core/discrete_space.hpp>

namespace sdm
{
    StochasticProcess::StochasticProcess()
    {
    }

    StochasticProcess::StochasticProcess(number nb_states) : state_space_(nb_states)
    {
    }

    StochasticProcess::StochasticProcess(const std::vector<double> &start_distrib) : state_space_(start_distrib.size())
    {
        this->setStartDistrib(start_distrib);
    }

    StochasticProcess::StochasticProcess(const Vector &start_distrib) : state_space_(start_distrib.size()), start_distrib_(start_distrib)
    {
    }

    StochasticProcess::StochasticProcess(const DiscreteSpace &state_space) : state_space_(state_space)
    {
    }
    StochasticProcess::StochasticProcess(const DiscreteSpace &state_space, const Vector &start_distrib) : state_space_(state_space), start_distrib_(start_distrib)
    {
    }

    void StochasticProcess::setStartDistrib(const std::vector<double> &start_distrib)
    {
        this->start_distrib_ = Vector(start_distrib.size());
        for (number i = 0; i < start_distrib.size(); i++)
        {
            this->start_distrib_[i] = start_distrib[i];
        }
    }
    void StochasticProcess::setStartDistrib(const Vector &start_distrib)
    {
        this->start_distrib_ = start_distrib;
    }

    const Vector &StochasticProcess::getStartDistrib() const
    {
        return this->start_distrib_;
    }

    number StochasticProcess::getState() const
    {
        return this->internal_state_;
    }

    const DiscreteSpace &StochasticProcess::getStateSpace() const
    {
        return this->state_space_;
    }

    number StochasticProcess::getNumStates() const
    {
        return this->getStateSpace().getNumElements();
    }

    number StochasticProcess::getStateIndex(const std::string &name) const
    {
        return this->getStateSpace().getElementIndex(name);
    }

    std::string StochasticProcess::getStateName(number idx) const
    {
        return this->getStateSpace().getElementName(idx);
    }

    void StochasticProcess::setStatesNames(const std::vector<std::string> &names)
    {
        this->state_space_.setElementsNames(names);
    }

    void StochasticProcess::setState(number state)
    {
        this->internal_state_ = state;
    }

} // namespace sdm