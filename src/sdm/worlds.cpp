#include <vector>
#include <sdm/worlds.hpp>

namespace sdm
{
    namespace world
    {
        std::vector<std::string> available()
        {
            return {"MDP", "BeliefMDP", "OccupancyMDP", "Extensive-MDP", "Extensive-BeliefMDP", "Extensive-OccupancyMDP", "Hierarchical-MDP", "Hierarchical-BeliefMDP", "Hierarchical-OccupancyMDP"};
        }
    }
}