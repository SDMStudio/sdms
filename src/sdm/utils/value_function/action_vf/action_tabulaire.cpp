#include <sdm/utils/value_function/action_vf/action_tabulaire.hpp>

namespace sdm
{
    ActionVFTabulaire::ActionVFTabulaire() {}

    ActionVFTabulaire::ActionVFTabulaire(const std::shared_ptr<SolvableByHSVI> &world) : ActionVFBase(world) {}

    std::shared_ptr<Action> ActionVFTabulaire::selectBestAction(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t)
    {
        std::shared_ptr<Action> best_action;
        double max = -std::numeric_limits<double>::max(), tmp;

        std::cout<<"Tabulair 0 "<<std::endl;

        auto space = this->world_->getActionSpaceAt(state, t);
        std::cout<<"Tabulair 2 "<<std::endl;

        for (const auto &action : *space)
        {
            std::cout<<"Tabulair 1 "<<std::endl;

            auto casted_action = action->toAction();
            if (max < (tmp = vf->template backup<double>(state, casted_action, t)))
            {
                best_action = casted_action;
                max = tmp;
            }
        }
        // std::cout<<"best action "<<best_action->str()<<", max value "<<max<<std::endl;
        return best_action;
    }
}