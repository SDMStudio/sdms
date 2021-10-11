
#pragma once

#include <sdm/types.hpp>
#include <sdm/spaces.hpp>
#include <sdm/core/reward.hpp>
#include <sdm/algorithms/planning/hsvi.hpp>
#include <sdm/utils/value_function/value_function.hpp>

/**
 * @namespace  sdm
 * @brief Namespace grouping all tools required for sequential decision making.
 */
namespace sdm
{
    class InteractiveWorld : public GymInterface
    {
    protected:
        std::shared_ptr<MDPInterface> world;

    public:
        InteractiveWorld(const std::shared_ptr<MDPInterface> &world);
    };
} // namespace sdm