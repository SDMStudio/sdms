#pragma once

#include <sdm/utils/struct/vector.hpp>
#include <sdm/utils/value_function/update_rule/vupdate_rule.hpp>

namespace sdm
{
    namespace update
    {
        class registry
        {
        protected:
            typedef std::map<std::string, std::shared_ptr<UpdateRuleInterface> (*)(std::shared_ptr<ValueFunctionInterface> value_function)> map_type;
            static map_type container;

        public:
            static std::vector<std::string> available();
            static std::shared_ptr<UpdateRuleInterface> make(std::string name, std::shared_ptr<ValueFunctionInterface> value_function);
        };

        template <class TUpdate>
        std::shared_ptr<UpdateRuleInterface> createInstance(std::shared_ptr<ValueFunctionInterface> value_function)
        {
            return std::make_shared<TUpdate>(value_function);
        }
    }
}
