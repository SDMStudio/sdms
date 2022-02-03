#ifdef WITH_CPLEX
#include <sdm/utils/value_function/action_selection/lp/action_maxplan_lp.hpp>
#include <sdm/utils/value_function/pwlc_value_function_interface.hpp>

#include <sdm/core/state/interface/belief_interface.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>

#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/world/base/mpomdp_interface.hpp>

namespace sdm
{

    ActionSelectionMaxplanLP::ActionSelectionMaxplanLP() {}

    ActionSelectionMaxplanLP::ActionSelectionMaxplanLP(const std::shared_ptr<SolvableByDP> &world, Config config) : MaxPlanSelectionBase(world), DecentralizedLP(world) {}

    Pair<std::shared_ptr<Action>, double> ActionSelectionMaxplanLP::computeGreedyActionAndValue(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<Hyperplane> &hyperplane, number t)
    {
        this->current_hyperplane = hyperplane;
        return this->createLP(vf, state, t);
    }

    void ActionSelectionMaxplanLP::createObjectiveFunction(const std::shared_ptr<ValueFunctionInterface> &value_function, const std::shared_ptr<State> &state, IloNumVarArray &var, IloObjective &obj, number t)
    {
        auto mpomdp = MaxPlanSelectionBase::getWorld()->getUnderlyingProblem();
        auto occupancy_state = state->toOccupancyState();

        number recover = 0;
        double weight = 0.0;

        // Go over all joint history
        for (const auto &joint_history : occupancy_state->getJointHistories())
        {
            // Go over all action
            for (const auto &action : *mpomdp->getActionSpace(t))
            {
                // Compute \sum_{x} s(o,x)* discount * [ r(x,o) + \sum_{x_,z_} p(x,u,x_,z_,) * next_hyperplan(<o,z_>,x_)]
                weight = this->getWeight(value_function, occupancy_state, joint_history, action->toAction(), this->current_hyperplane, t);

                //<! 1.b get variable a(u|o)
                recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action->toAction(), joint_history));

                //<! 1.c set coefficient of variable a(u|o) i.e., s(x,o)  [ r(x,u) + \gamma \sum_{x_,z_} P(x_,z_|x,u) * \hyperplan_i(x_,o_)  ]
                obj.setLinearCoef(var[recover], weight);
            } // for all u
        }     // for all o
    }
}

#endif