// #pragma once

// #include <sdm/utils/value_function/action_selection/action_selection_base.hpp>
// #include <sdm/utils/linear_programming/variable_naming.hpp>

// // #include "../../toulbar2/src/toulbar2lib.hpp"
// #include "toulbar2lib.hpp"

// #include <sdm/utils/linear_algebra/mapped_vector.hpp>

// namespace sdm
// {
//     class ActionSelectionSawtoothWCSP : public ActionSelectionBase, public VarNaming
//     {
//     public:
//         using TData = std::shared_ptr<State>;
        
//         ActionSelectionSawtoothWCSP();
//         ActionSelectionSawtoothWCSP(const std::shared_ptr<SolvableByHSVI>& world);
        
//         /**
//          * @brief Select the best action and the hyperplan at t+1 associated for a state at a precise time
//          * 
//          * @param const std::shared_ptr<ValueFunction>& vf : Value function
//          * @param const std::shared_ptr<State>& state : current state
//          * @param number t : time step
//          * @return  Pair<std::shared_ptr<Action>,TData> : best action and the hyperplan at t+1 associated
//          */
//         Pair<std::shared_ptr<Action>, double> getGreedyActionAndValue(const std::shared_ptr<ValueFunctionInterface>& vf, const std::shared_ptr<State>& state, number t);
    
//         // Fonction temporaire le temps de bien comprendre 
//         Pair<std::shared_ptr<Action>,double>  createWCSPProblem(const std::shared_ptr<ValueFunctionInterface>& vf, const std::shared_ptr<State>& state, number t);
    
//     protected :

//         /**
//          * @brief defines the maximum value in the domain of the payoff function
//          * 
//          */
//         double max;

//         /**
//          * @brief factor used to convert real values into integer costs
//          * 
//          */
//         long offset = 1000000000000;

//         /**
//          * @brief Returns a cost value
//          * 
//          * @param double defines a real value
//          */
//         Cost getCost(double);

//         double getValueAt(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<OccupancyStateInterface>& occupancy_state, const std::shared_ptr<JointHistoryInterface>& joint_history, const std::shared_ptr<Action>& action,const std::shared_ptr<State>& next_hyperplan, number t);

//         Pair<std::shared_ptr<State>,double> representation;
//         std::shared_ptr<HistoryInterface> support_of_the_next_history;
//         std::shared_ptr<State> support_of_the_next_hidden_state;
//         std::shared_ptr<Action> support_of_the_next_action;
//         bool support_empty;

//         std::unordered_map<std::shared_ptr<State>,std::shared_ptr<Action>> state_linked_to_decision_rule;

//         void determineMaxValue(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State>& state, number t);

//         std::set<std::shared_ptr<JointHistoryInterface>> determineJointHistory(const std::shared_ptr<State> &state);

//         void createWCSPVariable(std::shared_ptr<WeightedCSPSolver>&,const std::shared_ptr<State>& state,number t);
//         void createWCSPCostGraph(std::shared_ptr<WeightedCSPSolver>& wcsp_solver,const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state,const std::set<std::shared_ptr<JointHistoryInterface>>& set_joint_history,number t);
//         Pair<std::shared_ptr<Action>,double> getWSCPResult(std::shared_ptr<WeightedCSPSolver>& wcsp_solver,const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state,const std::set<std::shared_ptr<JointHistoryInterface>>& set_joint_history,number t);


//     };
// }
