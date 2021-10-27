#include <sdm/utils/value_function/action_selection/action_selection_base.hpp>
namespace sdm
{
    ActionSelectionBase::ActionSelectionBase() {}

    ActionSelectionBase::ActionSelectionBase(const std::shared_ptr<SolvableByDP> &world) : world_(world) {}

    ActionSelectionBase::~ActionSelectionBase() {}

    std::shared_ptr<SolvableByDP> ActionSelectionBase::getWorld() const
    {
        return this->world_;
    }

}