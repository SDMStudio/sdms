#pragma once

#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/value_function/action_selection/action_selection_interface.hpp>

namespace sdm
{
    class ActionSelectionBase : public ActionSelectionInterface
    {
    public:
        ActionSelectionBase();
        ActionSelectionBase(const std::shared_ptr<SolvableByDP> &world);
        virtual ~ActionSelectionBase();

        std::shared_ptr<SolvableByDP> getWorld() const;

    protected:
        std::shared_ptr<SolvableByDP> world_;
    };
}
