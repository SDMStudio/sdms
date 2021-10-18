#ifdef WITH_CPLEX

#pragma once

#include <sdm/utils/value_function/action_selection/action_selection_base.hpp>
#include <sdm/utils/linear_programming/decentralized_lp_problem.hpp>

namespace sdm
{

    class ConstraintProgrammingSelection : public ActionSelectionBase
    {
    public:

        ConstraintProgrammingSelection();
        
        ConstraintProgrammingSelection(const std::shared_ptr<SolvableByHSVI>& world);
        
        /**
         * @brief Select the best action and the hyperplan at t+1 associated for a state at a precise time
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return  Pair<std::shared_ptr<Action>,TData> : best action and the hyperplan at t+1 associated
         */
        Pair<std::shared_ptr<Action>, double> getGreedyActionAndValue(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t);

        /**
         * @brief Create and solve the constrained problem.
         * 
         * @param vf the value function
         * @param state the state
         * @param t the time step
         * @return action and corresponding value
         */
        virtual Pair<std::shared_ptr<Action>, double> createAndSolveCP(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t) = 0;
    };
}

#endif