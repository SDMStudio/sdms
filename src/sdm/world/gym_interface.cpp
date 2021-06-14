#include <sdm/world/gym_interface.hpp>

namespace sdm
{
    GymInterface::GymInterface(const std::shared_ptr<Space>& observation_space, const std::shared_ptr<Space>& action_space)
        : observation_space_(observation_space), action_space_(action_space)
    {
    }
} // namespace sdm