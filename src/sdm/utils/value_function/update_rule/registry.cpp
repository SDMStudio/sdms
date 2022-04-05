#include <sdm/macros.hpp>
#include <sdm/utils/value_function/update_rule.hpp>
#include <sdm/utils/value_function/update_rule/registry.hpp>

SDMS_REGISTRY(update)
SDMS_REGISTER("TabularUpdate", TabularUpdate)
SDMS_REGISTER("PWLCUpdate", PWLCUpdate)
SDMS_REGISTER("LBTabularUpdate", LowerBoundTabularUpdate)
SDMS_END_REGISTRY()

namespace sdm
{
    namespace update
    {
        std::vector<std::string> registry::available()
        {
            std::vector<std::string> available_init;
            for (auto const &pair_init : container)
                available_init.push_back(pair_init.first);
            return available_init;
        }

        std::shared_ptr<UpdateRuleInterface> registry::make(std::string name, std::shared_ptr<ValueFunctionInterface> value_function)
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
            return it->second(value_function);
        }
    }
}