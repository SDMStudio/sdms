#ifdef WITH_CPLEX
#include <sdm/utils/linear_programming/decentralized_lp_problem.hpp>

#include <sdm/world/base/mmdp_interface.hpp>
#include <sdm/world/serial_mmdp.hpp>

#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>

namespace sdm
{
    IndividualLP::IndividualLP() {}
    IndividualLP::IndividualLP(const std::shared_ptr<SolvableByDP> &world) : LPBase(world) {}

    void IndividualLP::createVariables(const std::shared_ptr<ValueFunctionInterface> &, const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t, number agent_id)
    {
        auto underlying_problem = std::dynamic_pointer_cast<MMDPInterface>(this->world_->getUnderlyingProblem());
        auto occupancy_state = state->toOccupancyState();

        //<! tracking variables
        std::string VarName;

        //<! 0.a Build variables  a_i(u_i|o_i)
        for (const auto &indiv_history : occupancy_state->getIndividualHistories(agent_id))
        {
            // Go over all Individual Action
            auto action_end_iter = underlying_problem->getActionSpace(agent_id, t)->end();
            for (auto action_iter = underlying_problem->getActionSpace(agent_id, t)->begin(); !action_iter->equal(action_end_iter); action_iter = action_iter->next())
            {
                auto serial_action = action_iter->getCurrent();
                //<! 0.c Build variables a_i(u_i|o_i)
                VarName = this->getVarNameIndividualHistoryDecisionRule(serial_action, indiv_history, agent_id);
                var.add(IloBoolVar(env, 0.0, 1.0, VarName.c_str()));
                this->setNumber(VarName, index++);
            }
        }
    }

    void IndividualLP::createConstraints(const std::shared_ptr<ValueFunctionInterface> &, const std::shared_ptr<State> &state, IloEnv &env, IloModel &model, IloRangeArray &con, IloNumVarArray &var, number &index, number t, number agent_id)
    {
        auto underlying_problem = std::dynamic_pointer_cast<MMDPInterface>(this->world_->getUnderlyingProblem());
        auto occupancy_state = state->toOccupancyState();

        number recover = 0;

        // 4. Build deterministic policy constraints
        for (const auto &indiv_history : occupancy_state->getIndividualHistories(agent_id))
        {
            //<! 4.a set constraint  \sum_{u_i} a_i(u_i|o_i) = 1
            con.add(IloRange(env, 1.0, 1.0));
            auto action_end_iter = underlying_problem->getActionSpace(agent_id, t)->end();
            for (auto action_iter = underlying_problem->getActionSpace(agent_id, t)->begin(); !action_iter->equal(action_end_iter); action_iter = action_iter->next())
            {
                auto serial_action = action_iter->getCurrent();
                recover = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(serial_action, indiv_history, agent_id));
                con[index].setLinearCoef(var[recover], +1.0);
            }
            //<! increment constraints
            index++;
        }
    }

    std::shared_ptr<Action> IndividualLP::getVariableResult(const std::shared_ptr<ValueFunctionInterface> &, const std::shared_ptr<State> &state, const IloCplex &cplex, const IloNumVarArray &var, number t, number agent_id)
    {
        number index = 0;
        std::vector<std::shared_ptr<Action>> actions;
        std::vector<std::shared_ptr<HistoryInterface>> indiv_histories;

        auto underlying_problem = std::dynamic_pointer_cast<MMDPInterface>(this->world_->getUnderlyingProblem());
        auto occupancy_state = state->toOccupancyState();

        // Go over all individual histories
        for (const auto &ihistory : occupancy_state->getIndividualHistories(agent_id))
        {
            indiv_histories.push_back(ihistory);

            // Go over all individual action

            auto action_end_iter = underlying_problem->getActionSpace(agent_id, t)->end();
            for (auto action_iter = underlying_problem->getActionSpace(agent_id, t)->begin(); !action_iter->equal(action_end_iter); action_iter = action_iter->next())
            {
                auto action = action_iter->getCurrent();
                index = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(action, ihistory, agent_id));

                // Add the variable in the vector only if the variable is true
                if (cplex.getValue(var[index]) + .5 >= 1)
                {
                    actions.push_back(action);
                }
            }
        }
        return std::make_shared<DeterministicDecisionRule>(indiv_histories, actions);
    }

}
#endif