#pragma once

#include <sdm/utils/value_function/action_vf/action_vf_base.hpp>

namespace sdm
{
    class ActionVFMaxplan : public ActionVFBase<std::shared_ptr<State>>
    {
    public:
        using TData = std::shared_ptr<State>;
        
        ActionVFMaxplan();
        ActionVFMaxplan(const std::shared_ptr<SolvableByHSVI>& world);
        
        /**
         * @brief Select the best action and the hyperplan at t+1 associated for a state at a precise time
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return  Pair<std::shared_ptr<Action>,TData> : best action and the hyperplan at t+1 associated
         */
        Pair<std::shared_ptr<Action>,TData> selectBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t);

    protected : 
        Pair<std::shared_ptr<Action>,TData> selectBestActionBelief(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t);
        Pair<std::shared_ptr<Action>,TData> selectBestActionOccupancy(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t);

        TData selectHyperplanKnowingAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state,const std::shared_ptr<Action>&, number t);

    };
}
