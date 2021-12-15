#include <sdm/macros.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/utils/value_function.hpp>
#include <sdm/utils/value_function/registry.hpp>
#include <sdm/utils/value_function/initializer/registry.hpp>
#include <sdm/utils/value_function/action_selection/registry.hpp>
#include <sdm/utils/value_function/update_operator/registry.hpp>
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
                    names = names + "\"" + v + "\" ";
                throw sdm::exception::Exception(name + " not registered. Available worlds are : " + names + "}");
            }

            std::shared_ptr<Initializer> initializer;
            auto init_opt_config = config.getOpt<Config>("initializer");
            auto init_opt_str = config.getOpt<std::string>("initializer");
            if (init_opt_config.has_value())
                initializer = sdm::initializer::registry::make(init_opt_config.value().get<std::string>("name"), world, init_opt_config.value());
            else if (init_opt_str.has_value())
                initializer = sdm::initializer::registry::make(init_opt_str.value(), world);

            std::shared_ptr<ActionSelectionInterface> action_selection;
            auto act_opt_config = config.getOpt<Config>("action_selection");
            auto act_opt_str = config.getOpt<std::string>("action_selection");
            if (act_opt_config.has_value())
                action_selection = sdm::action_selection::registry::make(act_opt_config.value().get<std::string>("name"), world, act_opt_config.value());
            else if (act_opt_str.has_value())
                action_selection = sdm::action_selection::registry::make(act_opt_str.value(), world);

            auto vf = it->second(world, initializer, action_selection, config);
            auto update_operator = sdm::update::registry::make(config.get("update_operator", std::string("TabularUpdate")), vf);
            vf->setUpdateOperator(update_operator);
            return vf;
        }

        std::shared_ptr<ValueFunction> registry::make(std::string name, const std::shared_ptr<SolvableByDP> &world,
                                                      const std::shared_ptr<Initializer> &intializer,
                                                      const std::shared_ptr<ActionSelectionInterface> &action,
                                                      Config config)
        {
            typename map_type::iterator it = registry::container.find(name);
            if (it == registry::container.end())
            {
                std::string names = "{";
                for (auto &v : registry::available())
                    names = names + "\"" + v + "\" ";
                throw sdm::exception::Exception(name + " not registered. Available worlds are : " + names + "}");
            }
            return it->second(world, intializer, action, config);
        }
    }
}