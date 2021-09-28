#include <vector>
#include <sdm/worlds.hpp>

namespace sdm
{
    namespace world
    {
        std::vector<std::string> available()
        {
            return {"MDP", "POMDP", "MMDP", "DecPOMDP", "Extensive-MDP", "Extensive-POMDP", "Extensive-DecPOMDP"};
        }
    }
}