#include <sdm/utils/value_function/action_vf/action_maxplan.hpp>

namespace sdm
{
    ActionVFMaxplan::ActionVFMaxplan(){}

    ActionVFMaxplan::ActionVFMaxplan(const std::shared_ptr<SolvableByHSVI>& world): ActionVFBase<std::shared_ptr<State>>(world) {}

    Pair<std::shared_ptr<Action>,std::shared_ptr<State>> ActionVFMaxplan::selectBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t)
    {
        // Definie local Variable
        double argmax_global = -std::numeric_limits<double>::max();  
        std::shared_ptr<Action> decision_max;
        std::shared_ptr<State> best_hyperplan;

        // // Go other the hyperplanes of decision step t+1
        // for(const auto &hyperplan : vf->getSupport(t+1))
        // {
        //     auto decision_and_value = this->selectBestActionKnowingNextHyperplan(vf,state,hyperplan,t);

        //     // Take the best deterministic decision rule and best hyperplan associated
        //     if(argmax_global < decision_and_value.second)
        //     {
        //         argmax_global = decision_and_value.second;
        //         decision_max = decision_and_value.first;
        //         best_hyperplan = hyperplan;
        //     }
        // }
        return std::make_pair(decision_max,best_hyperplan);
    }
}