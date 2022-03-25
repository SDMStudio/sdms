#pragma once

#include <sdm/utils/config.hpp>
#include <sdm/utils/struct/vector.hpp>
#include <sdm/world/solvable_by_dp.hpp>
#include <sdm/utils/value_function/initializer.hpp>

namespace sdm
{
    namespace initializer
    {
        class registry
        {
        protected:
            typedef std::map<std::string, std::shared_ptr<Initializer> (*)(std::shared_ptr<SolvableByDP> world, Config config)> map_type;
            static map_type container;

        public:
            static std::vector<std::string> available();
            static std::shared_ptr<Initializer> make(std::string name, std::shared_ptr<SolvableByDP> world, Config config = {});
        };

        template <class TInitializer>
        std::shared_ptr<Initializer> createInstance(std::shared_ptr<SolvableByDP> world, Config config)
        {
            return std::make_shared<TInitializer>(world, config);
        }
    }
}
