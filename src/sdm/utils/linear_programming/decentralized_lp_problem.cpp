#ifdef WITH_CPLEX
#include <sdm/utils/linear_programming/decentralized_lp_problem.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/world/base/mmdp_interface.hpp>

namespace sdm
{
    DecentralizedLP::DecentralizedLP() {}
    DecentralizedLP::DecentralizedLP(const std::shared_ptr<SolvableByDP> &world) : IndividualLP(world) {}

    void DecentralizedLP::createVariables(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t)
    {
        //Create Joint Decentralized Variable
        auto occupancy_state = state->toOccupancyState();

        //<! tracking variables
        std::string VarName;

        //<! 0.a Build variables a(u|o)
        for (const auto &joint_history : occupancy_state->getJointHistories())
        {
            for (const auto &action : *getWorld()->getUnderlyingProblem()->getActionSpace(t))
            {
                //< 0.b Build variables a(u|o)
                VarName = this->getVarNameJointHistoryDecisionRule(action->toAction(), joint_history);
                var.add(IloBoolVar(env, 0.0, 1.0, VarName.c_str()));
                this->setNumber(VarName, index++);
            }
        }

        for (auto agent = 0; agent < getWorld()->getUnderlyingProblem()->getNumAgents(); ++agent)
        {
            //Create Individual Decentralized Variable
            IndividualLP::createVariables(vf, state, env, var, index, t, agent);
        }
    }

    void DecentralizedLP::createConstraints(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloModel &model, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        auto underlying_problem = getWorld()->getUnderlyingProblem();

        //Create Joint Decentralized Constraints and Control Constraints
        auto occupancy_state = state->toOccupancyState();

        number recover;

        // Go over all joint history
        for (const auto &jhistory : occupancy_state->getJointHistories())
        {
            // Go over all action
            for (const auto &action : *underlying_problem->getActionSpace(t))
            {
                auto joint_action = std::static_pointer_cast<JointAction>(action->toAction());

                //<! 3.a set constraint a(u|o) >= \sum_i a_i(u_i|o_i) + 1 - n
                con.add(IloRange(env, 1 - underlying_problem->getNumAgents(), +IloInfinity));
                //<! 3.a.1 get variable a(u|o)
                recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action->toAction(), jhistory->toJointHistory()));
                //<! 3.a.2 set coefficient of variable a(u|o)
                con[index].setLinearCoef(var[recover], +1.0);

                for (number agent = 0; agent < underlying_problem->getNumAgents(); ++agent)
                {
                    //<! 3.a.3 get variables a_i(u_i|o_i)
                    recover = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(joint_action->get(agent), jhistory->getIndividualHistory(agent), agent));
                    //<! 3.a.4 set coefficient of variable a_i(u_i|o_i)
                    con[index].setLinearCoef(var[recover], -1.0);
                } // for all agent
                //<! increment constraints
                index++;
            } // for all u
        }     // for all o

        //<! 3.b set constraint a(u|o) <= a_i(u_i|o_i)

        // Go over all joint history
        for (const auto &jhistory : occupancy_state->getJointHistories())
        {
            // Go over all action
            for (const auto &action : *underlying_problem->getActionSpace(t))
            {
                auto joint_action = std::static_pointer_cast<JointAction>(action->toAction());

                // Go over agent
                for (number agent = 0; agent < underlying_problem->getNumAgents(); ++agent)
                {
                    // set constraint range
                    con.add(IloRange(env, -IloInfinity, 0.0));
                    //<! 3.b.1 get variable a(u|o)
                    recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(joint_action, jhistory));
                    //<! 3.b.2 set coefficient of variable a(u|o)
                    con[index].setLinearCoef(var[recover], +1.0);
                    //<! 3.b.3 get variable a_i(u_i|o_i) action
                    recover = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(joint_action->get(agent), jhistory->getIndividualHistory(agent), agent));
                    //<! 3.b.4 set coefficient of variable a_i(u_i|o_i)
                    con[index].setLinearCoef(var[recover], -1.0);
                    //<! increment constraints
                    index++;
                } // for all agent
            }     // for all u
        }         // for all o

        for (number agent = 0; agent < underlying_problem->getNumAgents(); ++agent)
        {
            //Create Individual Decentralized Constraints
            IndividualLP::createConstraints(vf, state, env, model, con, var, index, t, agent);
        }
    }

    std::shared_ptr<Action> DecentralizedLP::getVariableResult(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, const IloCplex &cplex, const IloNumVarArray &var, number t)
    {
        std::vector<std::shared_ptr<DeterministicDecisionRule>> actions;

        //Determine the element useful for create a JointDeterminiticDecisionRule
        for (number agent = 0; agent < this->getWorld()->getUnderlyingProblem()->getNumAgents(); agent++)
        {
            actions.push_back(std::dynamic_pointer_cast<DeterministicDecisionRule>(IndividualLP::getVariableResult(vf, state, cplex, var, t, agent)));
        }
        
        //Create the JointDeterminiticDecisionRule
        return std::make_shared<JointDeterministicDecisionRule>(actions, this->getWorld()->getUnderlyingProblem()->getActionSpace(t));
    }
}

#endif
