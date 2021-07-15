#include <sdm/world/serial_occupancy_mdp.hpp>

namespace sdm
{

    SerialOccupancyMDP::SerialOccupancyMDP()
    {
    }

    SerialOccupancyMDP::SerialOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &underlying_dpomdp,
                                                                     number memory, bool compression,
                                                                     bool store_states, bool store_actions, int batch_size)
        : OccupancyMDP(underlying_dpomdp, memory, compression, store_states, store_actions, batch_size)
    {
    }

    std::shared_ptr<Space> SerialOccupancyMDP::computeActionSpaceAt(const std::shared_ptr<State> &ostate, number t)
    {
        
    }

} // namespace sdm
