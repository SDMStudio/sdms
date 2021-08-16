#include <sdm/utils/value_function/action_vf/action_tabulaire.hpp>
#include <sdm/utils/value_function/backup/tabular_backup.hpp>

namespace sdm
{
    ActionVFTabulaire::ActionVFTabulaire() {}

    ActionVFTabulaire::ActionVFTabulaire(const std::shared_ptr<SolvableByHSVI> &world) : ActionVFBase(world){}

    Pair<std::shared_ptr<Action>, double> ActionVFTabulaire::selectBestAction(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t)
    {
        std::shared_ptr<Action> best_action;
        double max = -std::numeric_limits<double>::max(), tmp;

        for (const auto& it = this->world_->getActionSpaceAt(state, t)->begin() ; it != this->world_->getActionSpaceAt(state, t)->end();++it)
        {
            auto casted_action = (*it)->toAction();
            if (max < (tmp = vf->template backup<double>(state, casted_action, t)))
            {
                best_action = casted_action;
                max = tmp;
            }
        }
        return {best_action, max};
    }
}