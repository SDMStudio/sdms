#include <sdm/utils/linear_programming/decentralized_lp_problem.hpp>

namespace sdm
{
    DecentralizedLP::DecentralizedLP(){}
    DecentralizedLP::DecentralizedLP(const std::shared_ptr<SolvableByHSVI>& world) : LPBase(world) {}

    void DecentralizedLP::createDecentralizedConstraintsIndividual(const std::shared_ptr<State> &state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t, number agent_id)
    {
        auto under_pb = std::dynamic_pointer_cast<MMDPInterface>(this->world_->getUnderlyingProblem());
        auto occupancy_state = state->toOccupancyState();

        number recover = 0;

        // 4. Build deterministic policy constraints
        for (const auto& indiv_history : occupancy_state->getIndividualHistories(agent_id))
        {
            //<! 4.a set constraint  \sum_{u_i} a_i(u_i|o_i) = 1
            con.add(IloRange(env, 1.0, 1.0));
            for (const auto& serial_action : *under_pb->getActionSpace(agent_id,t))
            {
                recover = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(serial_action->toAction(), indiv_history, agent_id));
                con[index].setLinearCoef(var[recover], +1.0);
            }
            //<! increment constraints
            index++;
        }
    }

    void DecentralizedLP::createDecentralizedVariablesIndividual(const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t, number agent_id)
    {
        auto under_pb = std::dynamic_pointer_cast<MMDPInterface>(this->world_->getUnderlyingProblem());
        auto occupancy_state = state->toOccupancyState();

        //<! tracking variables
        std::string VarName;

        //<! 0.a Build variables  a_i(u_i|o_i)
        for (const auto& indiv_history : occupancy_state->getIndividualHistories(agent_id))
        {
            for (const auto& serial_action : *under_pb->getActionSpace(agent_id,t))
            {
                //<! 0.c Build variables a_i(u_i|o_i)
                VarName = this->getVarNameIndividualHistoryDecisionRule(serial_action->toAction(), indiv_history, agent_id);
                var.add(IloBoolVar(env, VarName.c_str()));
                this->setNumber(VarName, index++);
            }
        }
    }

    void DecentralizedLP::createDecentralizedConstraintsJoint(const std::shared_ptr<State> &state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        auto under_pb = std::dynamic_pointer_cast<MMDPInterface>(this->world_->getUnderlyingProblem());
        auto occupancy_state = state->toOccupancyState();

        number recover;

        for (const auto& jhistory : occupancy_state->getJointHistories())
        {
            for (const auto& action : *under_pb->getActionSpace(t))
            {
                auto joint_action = std::static_pointer_cast<Joint<std::shared_ptr<Action>>>(action->toAction());

                //<! 3.a set constraint a(u|o) >= \sum_i a_i(u_i|o_i) + 1 - n
                con.add(IloRange(env, 1 - under_pb->getNumAgents(), +IloInfinity));
                //<! 3.a.1 get variable a(u|o)
                recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action->toAction(), jhistory->toJointHistory()));
                //<! 3.a.2 set coefficient of variable a(u|o)
                con[index].setLinearCoef(var[recover], +1.0);
                for (number agent = 0; agent < under_pb->getNumAgents(); ++agent)
                {
                    //<! 3.a.3 get variables a_i(u_i|o_i)
                    recover = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(joint_action->get(agent)->toAction(), jhistory->getIndividualHistory(agent), agent));
                    //<! 3.a.4 set coefficient of variable a_i(u_i|o_i)
                    con[index].setLinearCoef(var[recover], -1.0);
                } // for all agent
                //<! increment constraints
                index++;
            } // for all u
        }   // for all o
    }

    void DecentralizedLP::createDecentralizedControlConstraints(const std::shared_ptr<State> &state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        auto under_pb = std::dynamic_pointer_cast<MMDPInterface>(this->world_->getUnderlyingProblem());
        auto occupancy_state = state->toOccupancyState();

        number recover;

        //<! 3.b set constraint a(u|o) <= a_i(u_i|o_i)
        for (const auto& jhistory : occupancy_state->getJointHistories())
            {
            for (const auto& action : *under_pb->getActionSpace(t))
            {
                auto joint_action = std::static_pointer_cast<Joint<std::shared_ptr<Action>>>(action->toAction());

                for (number agent = 0; agent < under_pb->getNumAgents(); ++agent)
                {
                    // set constraint range
                    con.add(IloRange(env, -IloInfinity, 0.0));
                    //<! 3.b.1 get variable a(u|o)
                    recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(joint_action->toAction(), jhistory));
                    //<! 3.b.2 set coefficient of variable a(u|o)
                    con[index].setLinearCoef(var[recover], +1.0);
                    //<! 3.b.3 get variable a_i(u_i|o_i)action
                    recover = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(joint_action->get(agent)->toAction(), jhistory->getIndividualHistory(agent), agent));
                    //<! 3.b.4 set coefficient of variable a_i(u_i|o_i)
                    con[index].setLinearCoef(var[recover], -1.0);
                    //<! increment constraints
                    index++;
                } // for all agent
            }   // for all u
        }     // for all o
    }

    void DecentralizedLP::createDecentralizedVariablesJoint(const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t)
    {
        auto under_pb = std::dynamic_pointer_cast<MMDPInterface>(this->world_->getUnderlyingProblem());
        auto occupancy_state = state->toOccupancyState();

        //<! tracking variables
        std::string VarName;

        //<! 0.a Build variables a(u|o)
        for (const auto& joint_history : occupancy_state->getJointHistories())
        {
            for (const auto& action : *under_pb->getActionSpace(t))
            {
                //< 0.b Build variables a(u|o)
                VarName = this->getVarNameJointHistoryDecisionRule(action->toAction(), joint_history);
                var.add(IloNumVar(env, 0.0, +IloInfinity, VarName.c_str()));
                this->setNumber(VarName, index++);
            }
        }
    }

    Pair<std::vector<std::shared_ptr<Item>>,std::vector<std::shared_ptr<Item>>> DecentralizedLP::getVariableResultIndividual(const std::shared_ptr<State> &state,const IloCplex &cplex, const IloNumVarArray &var, number t, number agent_id)
    {
        number index = 0;
        std::vector<std::shared_ptr<Item>> actions;
        std::vector<std::shared_ptr<Item>> indiv_histories;

        auto under_pb = std::dynamic_pointer_cast<MMDPInterface>(this->world_->getUnderlyingProblem());
        auto occupancy_state = state->toOccupancyState();

        // Go over all individual histories
        for(const auto& ihistory : occupancy_state->getIndividualHistories(agent_id))
        {
            indiv_histories.push_back(ihistory);

            // Go over all individual action
            for(const auto& action : *under_pb->getActionSpace(agent_id,t))
            {
                index = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(action->toAction(), ihistory, agent_id));
                
                // Add the variable in the vector only if the variable is true
                if( cplex.getValue(var[index]) + .5 >= 1 )
                {
                    actions.push_back(action);
                }
            }
        }
        return std::make_pair(actions,indiv_histories);
    }

}