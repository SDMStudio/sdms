#include <sdm/utils/value_function/action_vf/action_maxplan.hpp>

#include <sdm/world/base/pomdp_interface.hpp>
#include <sdm/core/state/interface/belief_interface.hpp>

namespace sdm
{
    ActionVFMaxplan::ActionVFMaxplan(){}

    ActionVFMaxplan::ActionVFMaxplan(const std::shared_ptr<SolvableByHSVI>& world): ActionVFBase(world) {}

    std::shared_ptr<Action> ActionVFMaxplan::selectBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t)
    {
        std::cout<<"Test 0 "<<std::endl;

        auto under_pb = std::dynamic_pointer_cast<POMDPInterface>(this->world_->getUnderlyingProblem());

        // Definie local Variable
        double argmax_global = -std::numeric_limits<double>::max(), value;  
        std::shared_ptr<Action> action_;

        auto action_space = this->world_->getActionSpaceAt(state,t);
        for (const auto &action : *action_space)
        {
            auto hyperplan = vf->template backup<std::shared_ptr<State>>(state,action->toAction(),t);
            // std::cout<<"Hyperplan "<<hyperplan->str()<<std::endl;
            if(argmax_global < (value = state->toBelief()->operator^(hyperplan->toBelief())) )
            {
                argmax_global = value;
                action_ = action->toAction();
            }
        }
        return action_;
    }
}