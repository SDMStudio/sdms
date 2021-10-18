#include <sdm/utils/value_function/action_selection/action_selection_base.hpp>
namespace sdm
{
    ActionSelectionBase::ActionSelectionBase() {}

    ActionSelectionBase::ActionSelectionBase(const std::shared_ptr<SolvableByHSVI> &world) : world_(world) {}

    ActionSelectionBase::~ActionSelectionBase() {}

    std::shared_ptr<SolvableByHSVI> ActionSelectionBase::getWorld() const
    {
        return this->world_;
    }

}