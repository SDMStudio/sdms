#include <sdm/macros.hpp>
#include <sdm/utils/value_function/initializer/registry.hpp>
#include <sdm/utils/value_function/initializer.hpp>

SDMS_REGISTRY(initializer)
SDMS_REGISTER("Zero", ZeroInitializer)
SDMS_REGISTER("Value", ValueInitializer)
SDMS_REGISTER("Min", MinInitializer)
SDMS_REGISTER("Blind", BlindInitializer)
SDMS_REGISTER("Max", MaxInitializer)
SDMS_REGISTER("Mdp", MDPInitializer)
SDMS_REGISTER("Pomdp", POMDPInitializer)
SDMS_END_REGISTRY()

namespace sdm
{
    namespace initializer
    {
        std::vector<std::string> registry::available()
        {
            std::vector<std::string> available_init;
            for (auto const &pair_init : container)
                available_init.push_back(pair_init.first);
            return available_init;
        }

        std::shared_ptr<Initializer> registry::make(std::string name, std::shared_ptr<SolvableByDP> world, Config config)
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