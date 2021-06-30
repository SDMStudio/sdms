#include <sdm/utils/value_function/action_vf/action_tabulaire.hpp>

namespace sdm
{
    ActionVFTabulaire::ActionVFTabulaire(){}

    ActionVFTabulaire::ActionVFTabulaire(const std::shared_ptr<SolvableByHSVI>& world): ActionVFBase<double>(world) {}

    Pair<std::shared_ptr<Action>,double> ActionVFTabulaire::selectBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t)
    {
        std::shared_ptr<Action> best_action;
        double max = -std::numeric_limits<double>::max(), tmp;
        
        auto space = this->world_->getActionSpaceAt(state, t);
        for (const auto &action : *space)
        {
            // std::cout<<"action "<<action->str()<<std::endl;
            auto casted_action = action->toAction();
            // std::cout<<"max value "<<max<<std::endl;
            if (max < (tmp = this->getQValueAt(vf,state, casted_action, t)))
            {
                best_action = casted_action;
                max = tmp;
            }
        }
        // std::cout<<"best action "<<best_action->str()<<", max value "<<max<<std::endl;
        return std::make_pair(best_action,max);
    }

    double ActionVFTabulaire::getQValueAt(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        // std::cout<<"reward "<<this->world_->getReward(state, action, t)<<std::endl;
        // std::cout<<"discount "<<this->world_->getDiscount(t)<<std::endl;
        // std::cout<<"expected "<<this->world_->getExpectedNextValue(vf->getptr(), state, action, t)<<std::endl;
        return this->world_->getReward(state, action, t) + this->world_->getDiscount(t) * this->world_->getExpectedNextValue(vf->getptr(), state, action, t);
    }
}