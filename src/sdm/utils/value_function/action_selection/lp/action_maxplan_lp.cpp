#ifdef WITH_CPLEX
#include <sdm/utils/value_function/action_selection/lp/action_maxplan_lp.hpp>

#include <sdm/core/state/interface/belief_interface.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>

#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/world/base/mpomdp_interface.hpp>

namespace sdm
{

    ActionSelectionMaxplanLP::ActionSelectionMaxplanLP() {}
    ActionSelectionMaxplanLP::ActionSelectionMaxplanLP(const std::shared_ptr<SolvableByHSVI> &world) : MaxPlanSelectionBase(world), DecentralizedLP(world) {}

    Pair<std::shared_ptr<Action>, double> ActionSelectionMaxplanLP::computeGreedyActionAndValue(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, number t)
    {
        return this->createLP(vf, state, t);
    }

    void ActionSelectionMaxplanLP::createVariables(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &occupancy_state, IloEnv &env, IloNumVarArray &var, number &index, number t)
    {
        // re-init the set of variable names
        this->variables.clear();

        //<! set decentralized decision rule variables
        this->createDecentralizedVariables(vf, occupancy_state, env, var, index, t);
    }

    double ActionSelectionMaxplanLP::beta(const std::shared_ptr<State> &x, const std::shared_ptr<JointHistoryInterface> &o, const std::shared_ptr<Action> &u, number t)
    {
        auto mpomdp = std::dynamic_pointer_cast<MPOMDPInterface>(this->getWorld()->getUnderlyingProblem());
        // Determine the reward for the hidden state and the action
        double factor = mpomdp->getReward(x, u, t);

        // Go over all reachable next hidden state
        for (const auto &y : mpomdp->getReachableStates(x, u, t))
        {
            // Go over all reachable observation
            for (const auto &z : mpomdp->getReachableObservations(x, u, y, t))
            {
                // Determine the next joint history conditionning to the observation
                auto o_ = o->expand(z->toObservation())->toJointHistory();
                factor += this->getWorld()->getDiscount(t) * this->tmp_representation->toOccupancyState()->getProbability(o_, y) * mpomdp->getDynamics(x, u, y, z, t);
            }
        }
        return factor;
    }

    double ActionSelectionMaxplanLP::getWeight(const std::shared_ptr<OccupancyStateInterface> occupancy_state, const std::shared_ptr<JointHistoryInterface> joint_history, const std::shared_ptr<Action> action, number t)
    {
        // Compute \sum_{x} s(o,x)* [ r(x,u) + discount * \sum_{x_,z_} p(x,u,x_,z_,) * next_hyperplan(<o,z_>,x_)]
        double weight = 0.0;

        // Go over all hidden state in the belief conditionning to a joint history
        for (const auto &state : occupancy_state->getBeliefAt(joint_history)->getStates())
        {
            weight += occupancy_state->getProbability(joint_history, state) * this->beta(state, joint_history, action, t);
            // weight += occupancy_state->getProbability(joint_history, state) * this->value_function->getQ(state, joint_history, action, t);
        }
        return weight;
    }

    void ActionSelectionMaxplanLP::createObjectiveFunction(const std::shared_ptr<ValueFunctionInterface> &, const std::shared_ptr<State> &state, IloNumVarArray &var, IloObjective &obj, number t)
    {
        auto mpomdp = std::dynamic_pointer_cast<MPOMDPInterface>(this->getWorld()->getUnderlyingProblem());
        auto occupancy_state = state->toOccupancyState();

        number recover = 0;
        double weight = 0.0, factor = 0.0;

        // Go over all joint history
        for (const auto &joint_history : occupancy_state->getJointHistories())
        {
            // Go over all action
            for (const auto &action : *mpomdp->getActionSpace(t))
            {
                // Compute \sum_{x} s(o,x)* discount * [ r(x,o) + \sum_{x_,z_} p(x,u,x_,z_,) * next_hyperplan(<o,z_>,x_)]
                weight = this->getWeight(occupancy_state, joint_history, action->toAction(), t);

                //<! 1.b get variable a(u|o)
                recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action->toAction(), joint_history));

                //<! 1.c set coefficient of variable a(u|o) i.e., s(x,o)  [ r(x,u) + \gamma \sum_{x_,z_} P(x_,z_|x,u) * \hyperplan_i(x_,o_)  ]
                obj.setLinearCoef(var[recover], weight);
            } // for all u
        }     // for all o
    }

    void ActionSelectionMaxplanLP::createConstraints(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloModel &, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        this->createDecentralizedConstraints(vf, state, env, con, var, index, t);
    }

    void ActionSelectionMaxplanLP::createDecentralizedVariables(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t)
    {
        return DecentralizedLP::createDecentralizedVariablesOccupancy(vf, state, env, var, index, t);
    }

    void ActionSelectionMaxplanLP::createDecentralizedConstraints(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        return DecentralizedLP::createDecentralizedConstraintsOccupancy(vf, state, env, con, var, index, t);
    }

    std::shared_ptr<Action> ActionSelectionMaxplanLP::getVariableResult(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, const IloCplex &cplex, const IloNumVarArray &var, number t)
    {
        return DecentralizedLP::getVariableResultOccupancy(vf, state, cplex, var, t);
    }

    std::shared_ptr<SolvableByHSVI> ActionSelectionMaxplanLP::getWorld() const
    {
        return DecentralizedLP::world_;
    }

}

#endif