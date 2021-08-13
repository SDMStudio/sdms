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

    Pair<std::shared_ptr<Action>, double> ActionVFTabulaire::selectBestActionRelaxed(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t)
    {
        std::shared_ptr<Action> best_action;
        double max = -std::numeric_limits<double>::max(), tmp;

        for (const auto& it = this->world_->getActionSpaceAt(state, t)->begin() ; it != this->world_->getActionSpaceAt(state, t)->end();++it)
        {
            auto casted_action = (*it)->toAction();
            // std::cout<<"Test ? "<<std::endl;
            tmp = std::static_pointer_cast<TabularBackup>(vf->backup_)->backup_relaxed(vf,state,casted_action,t);
            // std::cout<<"Value "<<tmp<<std::endl;
            if (max < tmp)
            {
                best_action = casted_action;
                max = tmp;
            }
        }
        return {best_action, max};
    }

    void ActionVFTabulaire::voidFunction(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t)
    {
        std::shared_ptr<Action> best_action;
        double max = -std::numeric_limits<double>::max(), tmp;

        auto space = this->world_->getActionSpaceAt(state, t);

        for (const auto& action_tmp : *space)
        {
            auto casted_action = action_tmp->toAction();
            tmp = std::static_pointer_cast<TabularBackup>(vf->backup_)->backup_relaxed(vf,state,casted_action,t);

            if (max < tmp)
            {
                best_action = casted_action;
                max = tmp;
            }
            std::cout<<"Tmp "<<tmp<<std::endl;
        }
    }
}