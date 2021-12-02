#include <sdm/macros.hpp>
#include <sdm/worlds.hpp>
#include <sdm/world/registry.hpp>

SDMS_REGISTRY(world)
SDMS_REGISTER("OccupancyMDP", OccupancyMDP)
SDMS_REGISTER("SerialOccupancyMDP", SerialOccupancyMDP)
SDMS_REGISTER("HierarchicalOccupancyMDP", HierarchicalOccupancyMDP)
SDMS_REGISTER("oMDP", OccupancyMDP)
SDMS_REGISTER("soMDP", SerialOccupancyMDP)
SDMS_REGISTER("hoMDP", HierarchicalOccupancyMDP)
SDMS_END_REGISTRY()

namespace sdm
{
    namespace world
    {
        std::vector<std::string> registry::available()
        {
            std::vector<std::string> available_init;
            for (auto const &pair_init : container)
                available_init.push_back(pair_init.first);
            return available_init;
        }

        std::shared_ptr<SolvableByDP> registry::make(std::string name, const std::shared_ptr<MPOMDPInterface>& mpomdp, Config config)
        {
            typename map_type::iterator it = registry::container.find(name);
            if (it == registry::container.end())
            {
                std::string names = "{";
                for (auto &v : registry::available())
                {
                    names = names + "\"" + v + "\" ";
                }
                throw sdm::exception::Exception(name + " not registered. Available worlds are : " + names + "}");
            }
            return it->second(mpomdp, config);
        }
    }
}