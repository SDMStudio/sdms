// #pragma once

// #include <sdm/utils/value_function/action_vf/action_vf_base.hpp>

// namespace sdm
// {
//     class ActionVFMaxplanSerial : public ActionVFBase
//     {
//     public:
//         using TData = std::shared_ptr<State>;
        
//         ActionVFMaxplanSerial();
//         ActionVFMaxplanSerial(const std::shared_ptr<SolvableByHSVI>& world);
        
//         /**
//          * @brief Select the best action and the hyperplan at t+1 associated for a state at a precise time
//          * 
//          * @param const std::shared_ptr<ValueFunction>& vf : Value function
//          * @param const std::shared_ptr<State>& state : current state
//          * @param number t : time step
//          * @return  Pair<std::shared_ptr<Action>,TData> : best action and the hyperplan at t+1 associated
//          */
//         std::shared_ptr<Action> selectBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t);

//         /**
//          * @brief Select the best action and value associated for a state at a precise time and a precise next hyperplan at t +1
//          * 
//          * @param const std::shared_ptr<ValueFunction>& vf : Value function
//          * @param const std::shared_ptr<State>& state : current state
//          * @param const std::shared_ptr<BeliefInterface>&
//          * @param number t : time step
//          * 
//          * @return  Pair<std::shared_ptr<Action>,double> : best action and the value associated
//          */
//         Pair<std::shared_ptr<Action>,double> selectBestActionKnowingNextHyperplan(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, const std::shared_ptr<State>& next_hyperplan, number t);

//         double evaluationOfHyperplanKnowingNextHyperplanAndDiscreteAction(const std::shared_ptr<State> &serial_occupancy_state, const std::shared_ptr<Action>& action, const std::shared_ptr<State>& next_step_hyperplan, number t);

//     };
// }
