#pragma once

#include <sdm/utils/value_function/action_vf/action_vf_base.hpp>

namespace sdm
{
    class ActionVFTabulaire : public ActionVFBase
    {
    public:
        using TData = double;

        ActionVFTabulaire();
        ActionVFTabulaire(const std::shared_ptr<SolvableByHSVI> &world);

        /**
         * @brief Select the best action for a state at a precise time
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return std::shared_ptr<Action> : Action
         */
        std::shared_ptr<Action> selectBestAction(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t);
    };
}