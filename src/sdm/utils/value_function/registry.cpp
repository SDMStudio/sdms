#include <sdm/utils/value_function/registry.hpp>
#include <sdm/config.hpp>
#include <sdm/macros.hpp>
#include <sdm/worlds.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/tools.hpp>
#include <sstream>

SDMS_REGISTRY(value)
SDMS_REGISTER("TabularValueFunction", TabularValueFunction)
SDMS_REGISTER("TabularValueFunction2", TabularValueFunction2)
SDMS_REGISTER("PWLCValueFunction", PWLCValueFunction)
SDMS_REGISTER("SawtoothValueFunction", SawtoothValueFunction)
SDMS_REGISTER("SawtoothValueFunction2", SawtoothValueFunction2)
SDMS_END_REGISTRY()

namespace sdm
{
    namespace value
    {
        std::vector<std::string> registry::available()
        {
            std::vector<std::string> available_init;
            for (auto const &pair_init : container)
                available_init.push_back(pair_init.first);
            return available_init;
        }

        std::shared_ptr<ValueFunction> registry::make(std::string name, std::shared_ptr<SolvableByDP> world, Config config)
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
            return it->second(world, nullptr, nullptr, nullptr, config);
        }

        std::shared_ptr<ValueFunction> registry::make(std::string name, const std::shared_ptr<SolvableByDP> &mpomdp,
                                                      const std::shared_ptr<Initializer> &intializer,
                                                      const std::shared_ptr<ActionSelectionInterface> &action,
                                                      const std::shared_ptr<UpdateOperatorInterface> &update_operator,
                                                      Config config)
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
            return it->second(mpomdp, intializer, action, update_operator, config);
        }
    }
}