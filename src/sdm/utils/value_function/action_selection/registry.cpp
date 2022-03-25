#include <sdm/macros.hpp>
#include <sdm/utils/value_function/action_selection/registry.hpp>
#include <sdm/utils/value_function/action_selection.hpp>

SDMS_REGISTRY(action_selection)
SDMS_REGISTER("Exhaustive", ExhaustiveActionSelection)
SDMS_REGISTER("MaxplanSerial", ActionSelectionMaxplanSerial)
SDMS_REGISTER("MaxplanWCSP", ActionSelectionMaxplanWCSP)
#ifdef WITH_CPLEX
SDMS_REGISTER("MaxplanLP", ActionSelectionMaxplanLP)
SDMS_REGISTER("SawtoothLP", ActionSelectionSawtoothLP)
SDMS_REGISTER("SawtoothLPSerial", ActionSelectionSawtoothLPSerial)
#endif
SDMS_END_REGISTRY()

namespace sdm
{
    namespace action_selection
    {
        std::vector<std::string> registry::available()
        {
            std::vector<std::string> available_init;
            for (auto const &pair_init : container)
                available_init.push_back(pair_init.first);
            return available_init;
        }

        std::shared_ptr<ActionSelectionInterface> registry::make(std::string name, std::shared_ptr<SolvableByDP> world, Config config)
        {
            typename map_type::iterator it = registry::container.find(name);
            if (it == registry::container.end())
            {
                std::string init_names = "{";
                for (auto &v : registry::available())
                {
                    init_names = init_names + "\"" + v + "\" ";
                }
                throw sdm::exception::Exception(name + " not registered. Available initializers are : " + init_names + "}");
            }
            return it->second(world, config);
        }
    }
}