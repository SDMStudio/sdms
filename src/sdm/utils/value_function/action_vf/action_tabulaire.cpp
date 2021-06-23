#include <sdm/utils/value_function/action_vf/action_tabulaire.hpp>

namespace sdm
{
    ActionVFTabulaire::ActionVFTabulaire(){}

    ActionVFTabulaire::ActionVFTabulaire(const std::shared_ptr<SolvableByHSVI>& world): ActionVFBase<double>(world) {}

    Pair<std::shared_ptr<Action>,double> ActionVFTabulaire::selectBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t)
    {
        std::shared_ptr<Action> best_action;
        double max = -std::numeric_limits<double>::max(), tmp;

        for (const auto &action : *this->world_->getActionSpaceAt(state, t))
        {
            auto casted_action = action->toAction();
            if (max < (tmp = this->getQValueAt(vf,state, casted_action, t)))
            {
                best_action = casted_action;
                max = tmp;
            }
        }
        return std::make_pair(best_action,max);
    }

    double ActionVFTabulaire::getQValueAt(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        return this->world_->getReward(state, action, t) + this->world_->getDiscount(t) * this->world_->getExpectedNextValue(vf->getptr(), state, action, t);
    }
}