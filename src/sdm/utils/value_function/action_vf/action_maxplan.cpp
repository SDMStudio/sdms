#include <sdm/utils/value_function/action_vf/action_maxplan.hpp>

#include <sdm/world/base/mpomdp_interface.hpp>
#include <sdm/world/serialized_mpomdp.hpp>
#include <sdm/world/belief_mdp.hpp>

#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/core/state/belief_state.hpp>
#include <sdm/core/state/occupancy_state.hpp>

#include <sdm/core/action/joint_det_decision_rule.hpp>

#include <sdm/utils/value_function/hyperplan_value_function.hpp>

namespace sdm
{
    ActionVFMaxplan::ActionVFMaxplan(){}

    ActionVFMaxplan::ActionVFMaxplan(const std::shared_ptr<SolvableByHSVI>& world): ActionVFBase(world) {}

    std::shared_ptr<Action> ActionVFMaxplan::selectBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t)
    {
        auto under_pb = std::dynamic_pointer_cast<POMDPInterface>(this->world_->getUnderlyingProblem());

        // Definie local Variable
        double argmax_global = -std::numeric_limits<double>::max(), value;  
        std::shared_ptr<Action> action_;

        auto action_space = this->world_->getActionSpaceAt(state,t);
        for (const auto &action : *action_space)
        {
            auto hyperplan = vf->template backup<std::shared_ptr<State>>(state,action->toAction(),t);

            if(argmax_global < (value = state->toBelief()->operator^(hyperplan->toBelief())) )
            {
                argmax_global = value;
                action_ = action->toAction();
            }
        }
        return action_;
    }
}