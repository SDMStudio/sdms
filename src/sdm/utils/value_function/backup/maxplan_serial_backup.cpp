#include <sdm/utils/value_function/backup/maxplan_serial_backup.hpp>
#include <sdm/world/serialized_mpomdp.hpp>

#include <sdm/core/action/det_decision_rule.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>

namespace sdm
{

    MaxPlanSerialBackup::MaxPlanSerialBackup() {}

    MaxPlanSerialBackup::MaxPlanSerialBackup(const std::shared_ptr<SolvableByHSVI> &world) : MaxPlanBackup(world)
    {}

    std::shared_ptr<State> MaxPlanSerialBackup::backup(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state,const std::shared_ptr<Action>& action, number t)
    {
        // auto pair_hyperplan_action = this->getBestActionAndMaxHyperplan(vf,state,t);
        // return this->setHyperplan(vf,state, pair_hyperplan_action.first->toBelief(),pair_hyperplan_action.second, t);
    }
} // namespace sdm