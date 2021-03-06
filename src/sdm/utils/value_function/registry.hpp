#pragma once

#include <sdm/utils/config.hpp>
#include <sdm/utils/struct/vector.hpp>
#include <sdm/world/solvable_by_dp.hpp>
#include <sdm/utils/value_function/initializer/initializer.hpp>
#include <sdm/utils/value_function/action_selection/action_selection_interface.hpp>
#include <sdm/utils/value_function/update_operator/vupdate_operator.hpp>

namespace sdm
{
    namespace value
    {
        class registry
        {
        protected:
            typedef std::map<std::string, std::shared_ptr<ValueFunction> (*)(std::shared_ptr<SolvableByDP> world,
                                                                             const std::shared_ptr<Initializer> &intializer,
                                                                             const std::shared_ptr<ActionSelectionInterface> &action,
                                                                             Config config)>
                map_type;

            static map_type container;

        public:
            static std::vector<std::string> available();
            static std::shared_ptr<ValueFunction> make(std::string name, std::shared_ptr<SolvableByDP> world, Config config = {});
            static std::shared_ptr<ValueFunction> make(std::string name, const std::shared_ptr<SolvableByDP> &world,
                                                       const std::shared_ptr<Initializer> &intializer = nullptr,
                                                       const std::shared_ptr<ActionSelectionInterface> &action = nullptr,
                                                       Config config = {});
        };

        template <class TFormalism>
        std::shared_ptr<ValueFunction> createInstance(std::shared_ptr<SolvableByDP> world,
                                                      const std::shared_ptr<Initializer> &intializer,
                                                      const std::shared_ptr<ActionSelectionInterface> &action,
                                                      Config config)
        {
            return std::make_shared<TFormalism>(world, intializer, action, config);
        }
    }
}
