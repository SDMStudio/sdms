#include <sdm/world/gym_interface.hpp>

namespace sdm
{

    GymInterface::GymInterface(std::shared_ptr<Space> obs_space, std::shared_ptr<Space>  act_space) : observation_space_(obs_space), action_space_(act_space)
    {
    }

    // GymInterface::GymInterface(MultiDiscreteSpace obs_space, MultiDiscreteSpace act_space) : observation_space_(obs_space), action_space_(act_space)
    // {
    // }

    std::shared_ptr<Space> GymInterface::getObsSpace() const
    {
        return this->observation_space_;
    }
    std::shared_ptr<Space> GymInterface::getActionSpace() const
    {
        return this->action_space_;
    }
} // namespace sdm