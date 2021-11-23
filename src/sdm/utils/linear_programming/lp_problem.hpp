#ifdef WITH_CPLEX
#pragma once
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/core/state/interface/belief_interface.hpp>
#include <sdm/utils/linear_programming/variable_naming.hpp>
#include <sdm/utils/linear_programming/lp_problem_interface.hpp>

namespace sdm
{
    class LPBase : public LPInterface, public VarNaming
    {
    public:
        LPBase();
        LPBase(const std::shared_ptr<SolvableByDP> &);
        ~LPBase();

        /**
         * @brief Get the world 
         */
        std::shared_ptr<SolvableByDP> getWorld() const;

        /**
         * @brief Main function who is used to create the Linear program and solve it.
         * 
         * @param occupancy_state the occupancy state
         * @param t the time step
         * @return the decision rule 
         */
        Pair<std::shared_ptr<Action>, double> createLP(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &occupancy_state, number t);

    protected:
        /**
         * @brief The world
         */
        std::shared_ptr<SolvableByDP> world_;
    };
}
#endif