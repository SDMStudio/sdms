#pragma once

#include <sdm/utils/config.hpp>
#include <sdm/utils/struct/vector.hpp>
#include <sdm/world/gym_interface.hpp>

namespace sdm
{
    namespace world
    {

        namespace gym
        {
            class registry
            {
            protected:
                typedef std::map<std::string, std::shared_ptr<GymInterface> (*)(Config config)> map_type;
                static map_type container;

            public:
                static std::vector<std::string> available();
                static std::shared_ptr<GymInterface> make(std::string name, Config config = Config());
            };

            template <class TWorld>
            std::shared_ptr<GymInterface> createInstance(Config config)
            {
                return std::make_shared<TWorld>(config);
            }
        }
    }
}
