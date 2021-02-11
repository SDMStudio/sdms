#include <sdm/world/stochastic_process.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/common.hpp>

namespace sdm
{
    StochasticProcess::StochasticProcess()
    {
    }

    // StochasticProcess::StochasticProcess(number nb_states) : state_space_(nb_states)
    // {
    // }

    // StochasticProcess::StochasticProcess(const std::vector<double> &start_distrib) : state_space_(start_distrib.size())
    // {
    //     this->setStartDistrib(start_distrib);
    // }

    // StochasticProcess::StochasticProcess(const Vector &start_distrib) : state_space_(start_distrib.size()), start_distrib_(start_distrib)
    // {
    // }

    StochasticProcess::StochasticProcess(DiscreteSpace<number> state_space) : state_space_(state_space)
    {
    }
    StochasticProcess::StochasticProcess(DiscreteSpace<number> state_space, Vector start_distrib) : state_space_(state_space), start_distrib_(start_distrib)
    {
    }

    number StochasticProcess::init()
    {
        this->internal_state_ = this->start_generator(sdm::common::global_urng());
        return this->internal_state_;
    }

    void StochasticProcess::setupStartGenerator()
    {
        std::vector<double> v;
        for (number x = 0; x < this->getNumStates(); ++x)
            v.push_back(this->getStartDistrib()[x]);
        this->start_generator = std::discrete_distribution<number>(v.begin(), v.end());
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

    number StochasticProcess::getInternalState() const
    {
        return this->internal_state_;
    }

    DiscreteSpace<number> StochasticProcess::getStateSpace() const
    {
        return this->state_space_;
    }

    number StochasticProcess::getNumStates() const
    {
        return this->getStateSpace().getNumItems();
    }

    void StochasticProcess::setInternalState(number state)
    {
        this->internal_state_ = state;
    }

} // namespace sdm