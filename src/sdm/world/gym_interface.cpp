#include <sdm/world/gym_interface.hpp>

namespace sdm
{
    template <typename TObsSpace, typename TActSpace>
    GymInterface<TObsSpace, TActSpace>::GymInterface(std::shared_ptr<TObsSpace> obs_space, std::shared_ptr<TActSpace> act_space) : observation_space_(obs_space), action_space_(act_space)
    {
    }

    // GymInterface<TObsSpace, TActSpace>::GymInterface(MultiDiscreteSpace obs_space, MultiDiscreteSpace act_space) : observation_space_(obs_space), action_space_(act_space)
    // {
    // }

    template <typename TObsSpace, typename TActSpace>
    std::shared_ptr<TObsSpace> GymInterface<TObsSpace, TActSpace>::getObsSpace() const
    {
        return this->observation_space_;
    }

    template <typename TObsSpace, typename TActSpace>
    std::shared_ptr<TActSpace> GymInterface<TObsSpace, TActSpace>::getActionSpace() const
    {
        return this->action_space_;
    }
} // namespace sdm