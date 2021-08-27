#pragma once

#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/core/state/interface/belief_interface.hpp>
#include <sdm/utils/linear_programming/lp_problem_interface.hpp>
// #include <sdm/utils/linear_algebra/vector_interface.hpp>

namespace sdm
{
    class LPBase : public LPInterface
    {
    public:
        LPBase();
        LPBase(const std::shared_ptr<SolvableByHSVI>&);
        ~LPBase();

        /**
         * @brief Main function who is used to create the Linear program
         * 
         * @param occupancy_state 
         * @param t 
         * @return Pair<std::shared_ptr<Action>,double> 
         */
        Pair<std::shared_ptr<Action>,double> createLP(const std::shared_ptr<ValueFunction>&vf, const std::shared_ptr<State> &occupancy_state, number t);


    protected : 
    
        std::shared_ptr<SolvableByHSVI> world_;

        /**
         * @brief The temporary one-stage value function represention.
         */
        std::shared_ptr<BeliefInterface> tmp_representation;
    };
}