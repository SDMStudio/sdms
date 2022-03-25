#pragma once

#include <sdm/utils/struct/vector.hpp>
#include <sdm/utils/value_function/action_selection.hpp>

namespace sdm
{
    namespace action_selection
    {
        class registry
        {
        protected:
            typedef std::map<std::string, std::shared_ptr<ActionSelectionInterface> (*)(std::shared_ptr<SolvableByDP> world, Config config)> map_type;
            static map_type container;

        public:
            static std::vector<std::string> available();
            static std::shared_ptr<ActionSelectionInterface> make(std::string name, std::shared_ptr<SolvableByDP> world, Config config = {});
        };

        template <class TSelection>
        std::shared_ptr<ActionSelectionInterface> createInstance(std::shared_ptr<SolvableByDP> world, Config config)
        {
            return std::make_shared<TSelection>(world, config);
        }
    }
}
