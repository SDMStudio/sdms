#include <sdm/utils/value_function/action_vf/action_tabulaire.hpp>

namespace sdm
{
    long ActionVFTabulaire::MemoryUsed = 0;

    ActionVFTabulaire::ActionVFTabulaire() {}

    ActionVFTabulaire::ActionVFTabulaire(const std::shared_ptr<SolvableByHSVI> &world) : ActionVFBase(world){}

    Pair<std::shared_ptr<Action>, double> ActionVFTabulaire::selectBestAction(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t)
    {
        // struct sysinfo memInfo;
        // auto memory_start = std::Performance::RanMemoryUsed(memInfo);
        // this->voidFunction(vf, state, t);
        // auto memory_used = std::Performance::RanMemoryUsed(memInfo) - memory_start;
        // ActionVFTabulaire::MemoryUsed += memory_used;
        // std::cout<<"Memory Used 0 : "<<memory_used<<std::endl;

        // if(memory_used>0 && !state->isBaseItem() && state->getTypeState() == TypeState::OCCUPANCY_STATE)
        // {
        //     exit(-1);
        // }

        std::shared_ptr<Action> best_action;
        double max = -std::numeric_limits<double>::max(), tmp;

        auto space = this->world_->getActionSpaceAt(state, t);

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

    void ActionVFTabulaire::voidFunction(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t)
    {
        std::shared_ptr<Action> best_action;
        double max = -std::numeric_limits<double>::max(), tmp;

        auto space = this->world_->getActionSpaceAt(state, t);

        for (const auto& action_tmp : *space)
        {
            auto casted_action = action_tmp->toAction();
            if (max < (tmp = vf->template backup<double>(state, casted_action, t)))
            {
                best_action = casted_action;
                max = tmp;
            }
            // std::cout<<"Tmp "<<tmp<<std::endl;
        }
    }
}