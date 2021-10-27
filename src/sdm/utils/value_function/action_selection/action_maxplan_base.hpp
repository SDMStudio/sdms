#ifdef WITH_CPLEX

#pragma once

#include <sdm/utils/value_function/action_selection/action_selection_base.hpp>
#include <sdm/utils/linear_programming/decentralized_lp_problem.hpp>

namespace sdm
{

    class MaxPlanSelectionBase : public ActionSelectionBase
    {
    public:
        MaxPlanSelectionBase();
        MaxPlanSelectionBase(const std::shared_ptr<SolvableByDP> &world);

        /**
         * @brief Select the best action and the hyperplan at t+1 associated for a state at a precise time
         *
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return  Pair<std::shared_ptr<Action>,TData> : best action and the hyperplan at t+1 associated
         */
        Pair<std::shared_ptr<Action>, double> getGreedyActionAndValue(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, number t);

        /**
         * @brief Compute the greedy action and corresponding value for a specific next hyperplan (saved in the temporary representation).
         *
         * @param vf the value function
         * @param state the state
         * @param t the time step
         * @return action and corresponding value
         */
        virtual Pair<std::shared_ptr<Action>, double> computeGreedyActionAndValue(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, number t) = 0;

    protected:
        /**
         * @brief The temporary one-stage value function represention.
         */
        std::shared_ptr<BeliefInterface> tmp_representation;
    };
}

#endif