#pragma once

#include "toulbar2lib.hpp"

#include <sdm/utils/linear_programming/variable_naming.hpp>
#include <sdm/utils/value_function/action_selection/action_maxplan_base.hpp>

#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/world/base/mpomdp_interface.hpp>
namespace sdm
{
    class ActionSelectionMaxplanWCSP : public MaxPlanSelectionBase, public VarNaming
    {
    public:
        using TData = std::shared_ptr<State>;

        ActionSelectionMaxplanWCSP();
        ActionSelectionMaxplanWCSP(const std::shared_ptr<SolvableByDP> &world, Config config = {});

        /**
         * @brief Compute the greedy action and corresponding value for a specific next hyperplan (saved in the temporary representation).
         *
         * @param vf the value function
         * @param state the state
         * @param t the time step
         * @return action and corresponding value
         */
        Pair<std::shared_ptr<Action>, double> computeGreedyActionAndValue(const std::shared_ptr<ValueFunctionInterface> &value_function, const std::shared_ptr<State> &state, const std::shared_ptr<Hyperplane> &hyperplane, number t);

        Pair<std::shared_ptr<Action>, double> createAndSolveWCSP(const std::shared_ptr<ValueFunctionInterface> &value_function, const std::shared_ptr<State> &state, const std::shared_ptr<Hyperplane> &hyperplane, number t);

    protected:
        
        std::shared_ptr<OccupancyMDP> occupancy_mdp;
        std::shared_ptr<MMDPInterface> underlying_problem;

        /**
         * @brief defines the maximum value in the domain of the payoff function
         * 
         */
        double max;

        /**
         * @brief factor used to convert real values into integer costs
         * 
         */
        long offset = 1000000000000;

        /**
         * @brief Returns a cost value
         * 
         * @param double defines a real value
         */
        long getCost(double);

        void determineMaxValue(const std::shared_ptr<ValueFunctionInterface> &value_function, const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<Hyperplane> &hyperplane, number t);
    };
}
