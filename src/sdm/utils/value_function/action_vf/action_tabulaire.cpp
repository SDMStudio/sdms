#include <sdm/utils/value_function/action_vf/action_tabulaire.hpp>
#include <sdm/utils/value_function/backup/tabular_backup.hpp>
#include <sdm/core/state/occupancy_state.hpp>

namespace sdm
{
    ActionVFTabulaire::ActionVFTabulaire() {}

    ActionVFTabulaire::ActionVFTabulaire(const std::shared_ptr<SolvableByHSVI> &world) : ActionVFBase(world) {}

    Pair<std::shared_ptr<Action>, double> ActionVFTabulaire::selectBestAction(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t)
    {
        std::shared_ptr<Action> best_action;
        double max = -std::numeric_limits<double>::max(), tmp;

        auto action_space = this->world_->getActionSpaceAt(state, t);
        //Go over all Action Space
        for (const auto &action : *action_space)
        {

            //Determine the value of the backup for a precise action
            auto casted_action =   action->toAction();
            if (max < (tmp = vf->template backup<double>(state, casted_action, t)))
            {
                best_action = casted_action;
                max = tmp;
            }
        }
        //Return the best action and the value associated
        return {best_action, max};
    }
}