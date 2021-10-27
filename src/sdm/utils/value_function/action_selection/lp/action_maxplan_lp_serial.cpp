#ifdef WITH_CPLEX

#include <sdm/utils/value_function/action_selection/lp/action_maxplan_lp_serial.hpp>
#include <sdm/utils/value_function/pwlc_value_function_interface.hpp>

#include <sdm/world/serial_occupancy_mdp.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>
#include <sdm/world/serial_mpomdp.hpp>

namespace sdm
{

    ActionSelectionMaxplanLPSerial::ActionSelectionMaxplanLPSerial() {}
    ActionSelectionMaxplanLPSerial::ActionSelectionMaxplanLPSerial(const std::shared_ptr<SolvableByDP> &world) : ActionSelectionMaxplanLP(world) {}

    void ActionSelectionMaxplanLPSerial::createObjectiveFunction(const std::shared_ptr<ValueFunctionInterface> &value_function, const std::shared_ptr<State> &state, IloNumVarArray &var, IloObjective &obj, number t)
    {
        auto under_pb = std::dynamic_pointer_cast<SerialMPOMDP>(ActionSelectionBase::world_->getUnderlyingProblem());
        number agent_id = under_pb->getAgentId(t);
        auto occupancy_state = state->toOccupancyState();

        number recover = 0;
        double weight = 0.0, factor = 0.0;

        // Go over all joint history
        for (const auto &indiv_history : occupancy_state->getIndividualHistories(agent_id))
        {
            // Go over all action
            for (const auto &action : *under_pb->getActionSpace(t))
            {
                // Compute \sum_{x} s(o,x)* discount * [ r(x,o) + \sum_{x_,z_} p(x,u,x_,z_,) * next_hyperplan(<o,z_>,x_)]
                weight = 0.0;
                for (const auto &private_joint_history : std::dynamic_pointer_cast<OccupancyState>(occupancy_state)->getPrivateOccupancyState(agent_id, indiv_history)->getJointHistories())
                {
                    weight += this->getWeight(value_function, occupancy_state, private_joint_history, action->toAction(), t);
                }

                //<! 1.b get variable a(u|o)
                recover = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(action->toAction(), indiv_history, agent_id));

                //<! 1.c set coefficient of variable a(u|o) i.e., s(x,o)  [ r(x,u) + \gamma \sum_{x_,z_} P(x_,z_|x,u) * \hyperplan_i(x_,o_)  ]
                obj.setLinearCoef(var[recover], weight * occupancy_state->getProbabilityOverIndividualHistories(agent_id, indiv_history));
            } // for all u
        }     // for all o
    }

    void ActionSelectionMaxplanLPSerial::createDecentralizedVariables(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t)
    {
        //Determine the Agent
        auto under_pb = std::dynamic_pointer_cast<SerialMMDP>(ActionSelectionBase::world_->getUnderlyingProblem());
        number agent_id = under_pb->getAgentId(t);

        //Create Individual Decentralized Variable
        this->createDecentralizedVariablesIndividual(vf, state, env, var, index, t, agent_id);
    }

    void ActionSelectionMaxplanLPSerial::createDecentralizedConstraints(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        //Determine the Agent
        auto under_pb = std::dynamic_pointer_cast<SerialMMDP>(ActionSelectionBase::world_->getUnderlyingProblem());
        number agent_id = under_pb->getAgentId(t);

        //Create Individual Decentralized Constraints
        this->createDecentralizedConstraintsIndividual(vf, state, env, con, var, index, t, agent_id);
    }

    std::shared_ptr<Action> ActionSelectionMaxplanLPSerial::getVariableResult(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, const IloCplex &cplex, const IloNumVarArray &var, number t)
    {
        //Determine the Agent
        auto under_pb = std::dynamic_pointer_cast<SerialMMDP>(ActionSelectionBase::world_->getUnderlyingProblem());
        number agent_id = under_pb->getAgentId(t);

        //Determine the element useful for create a DeterminiticDecisionRule
        auto action_and_history_individual = this->getVariableResultIndividual(vf, state, cplex, var, t, agent_id);

        //Create the DeterminiticDecisionRule
        return std::make_shared<DeterministicDecisionRule>(action_and_history_individual.second, action_and_history_individual.first);
    }
}

#endif