#pragma once

#include <sdm/utils/struct/vector.hpp>
#include <sdm/world/solvable_by_dp.hpp>
#include <sdm/world/base/mpomdp_interface.hpp>

namespace sdm
{
    namespace world
    {
        class registry
        {
        protected:
            typedef std::map<std::string, std::shared_ptr<SolvableByDP> (*)(std::shared_ptr<MPOMDPInterface> mpomdp, Config config)> map_type;
            static map_type container;

        public:
            static std::vector<std::string> available();
            static std::shared_ptr<SolvableByDP> make(std::string name, const std::shared_ptr<MPOMDPInterface> &mpomdp, Config config = Config());
        };

        template <class TUpdate>
        std::shared_ptr<SolvableByDP> createInstance(std::shared_ptr<MPOMDPInterface> mpomdp, Config config)
        {
            return std::make_shared<TUpdate>(mpomdp, config);
        }
    }
}
