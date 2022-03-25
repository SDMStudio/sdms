#pragma once

#include <sdm/utils/config.hpp>
#include <sdm/utils/struct/vector.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/world/base/mpomdp_interface.hpp>

namespace sdm
{
    namespace formalism
    {
        class registry
        {
        protected:
            typedef std::map<std::string, std::shared_ptr<SolvableByHSVI> (*)(std::shared_ptr<MPOMDPInterface> mpomdp, Config config)> map_type;
            static map_type container;

        public:
            static std::vector<std::string> available();
            static std::shared_ptr<SolvableByHSVI> make(std::string name, Config config = {});
            static std::shared_ptr<SolvableByHSVI> make(std::string name, const std::shared_ptr<MPOMDPInterface> &mpomdp, Config config = {});
        };

        template <class TFormalism>
        std::shared_ptr<SolvableByHSVI> createInstance(std::shared_ptr<MPOMDPInterface> mpomdp, Config config)
        {
            if (mpomdp == nullptr)
                return std::make_shared<TFormalism>(config);
            else
                return std::make_shared<TFormalism>(mpomdp, config);
        }
    }

    namespace world
    {
        class registry
        {
        protected:
            typedef std::map<std::string, std::shared_ptr<MDPInterface> (*)(Config config)> map_type;
            static map_type container;

        public:
            static std::vector<std::string> available();
            static std::shared_ptr<MDPInterface> make(std::string name, Config config = Config());
        };

        template <class TWorld>
        std::shared_ptr<MDPInterface> createInstance(Config config)
        {
            return std::make_shared<TWorld>(config);
        }

        std::shared_ptr<MDPInterface> createFromConfig(Config config);

    }
}
