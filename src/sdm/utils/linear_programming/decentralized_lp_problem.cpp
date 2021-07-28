#include <sdm/utils/linear_programming/decentralized_lp_problem.hpp>

#include <sdm/world/base/mmdp_interface.hpp>
#include <sdm/world/serialized_mmdp.hpp>

#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>
namespace sdm
{
    DecentralizedLP::DecentralizedLP(){}
    DecentralizedLP::DecentralizedLP(const std::shared_ptr<SolvableByHSVI>& world) : LPBase(world) {}

    void DecentralizedLP::createDecentralizedConstraintsIndividual(const std::shared_ptr<ValueFunction>&,const std::shared_ptr<State> &state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t, number agent_id)
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

    void DecentralizedLP::createDecentralizedVariablesIndividual(const std::shared_ptr<ValueFunction>&,const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t, number agent_id)
    {
        auto under_pb = std::dynamic_pointer_cast<MMDPInterface>(this->world_->getUnderlyingProblem());
        auto occupancy_state = state->toOccupancyState();

        //<! tracking variables
        std::string VarName;

        //<! 0.a Build variables  a_i(u_i|o_i)
        for (const auto& indiv_history : occupancy_state->getIndividualHistories(agent_id))
        {
            // Go over all Individual Action
            for (const auto& serial_action : *under_pb->getActionSpace(agent_id,t))
            {
                //<! 0.c Build variables a_i(u_i|o_i)
                VarName = this->getVarNameIndividualHistoryDecisionRule(serial_action->toAction(), indiv_history, agent_id);
                var.add(IloBoolVar(env, 0.0, 1.0, VarName.c_str()));
                this->setNumber(VarName, index++);
            }
        }
    }

    void DecentralizedLP::createDecentralizedConstraintsJoint(const std::shared_ptr<ValueFunction>&, const std::shared_ptr<State> &state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        auto under_pb = std::dynamic_pointer_cast<MMDPInterface>(this->world_->getUnderlyingProblem());
        auto occupancy_state = state->toOccupancyState();

        number recover;

        // Go over all joint history 
        for (const auto& jhistory : occupancy_state->getJointHistories())
        {
            // Go over all action
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

    void DecentralizedLP::createDecentralizedControlConstraints(const std::shared_ptr<ValueFunction>&, const std::shared_ptr<State> &state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        auto under_pb = std::dynamic_pointer_cast<MMDPInterface>(this->world_->getUnderlyingProblem());
        auto occupancy_state = state->toOccupancyState();

        number recover;

        //<! 3.b set constraint a(u|o) <= a_i(u_i|o_i)

        // Go over all joint history
        for (const auto& jhistory : occupancy_state->getJointHistories())
        {
            // Go over all action
            for (const auto& action : *under_pb->getActionSpace(t))
            {
                auto joint_action = std::static_pointer_cast<Joint<std::shared_ptr<Action>>>(action->toAction());

                // Go over agent
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

    void DecentralizedLP::createDecentralizedVariablesJoint(const std::shared_ptr<ValueFunction>&, const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t)
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
                var.add(IloBoolVar(env, 0.0, 1.0, VarName.c_str()));
                this->setNumber(VarName, index++);
            }
        }
    }

    Pair<std::vector<std::shared_ptr<Item>>,std::vector<std::shared_ptr<Item>>> DecentralizedLP::getVariableResultIndividual(const std::shared_ptr<ValueFunction>&, const std::shared_ptr<State> &state,const IloCplex &cplex, const IloNumVarArray &var, number t, number agent_id)
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


    void DecentralizedLP::createDecentralizedVariables(const std::shared_ptr<ValueFunction>&vf,const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t)
    {
        //Specialisation for type of state
        switch (state->getTypeState())
        {
        case TypeState::OCCUPANCY_STATE :
            this->createDecentralizedVariablesOccupancy(vf,state,env,var,index,t);
            break;
        case TypeState::SERIAL_OCCUPANCY_STATE :
            this->createDecentralizedVariablesSerial(vf,state,env,var,index,t);
        
        default:
            throw sdm::exception::Exception("DecentralizedLP::createDecentralizedVariables This state is not defined for this function");
            break;
        }
    }

    void DecentralizedLP::createDecentralizedConstraints(const std::shared_ptr<ValueFunction>&vf,const std::shared_ptr<State>& state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        // std::cout << "DecentralizedLP::createDecentralizedConstraints"<<std::endl;
        //Specialisation for type of state
        switch (state->getTypeState())
        {
        case TypeState::OCCUPANCY_STATE :
            this->createDecentralizedConstraintsOccupancy(vf,state,env,con,var,index,t);
            break;
        case TypeState::SERIAL_OCCUPANCY_STATE :
            this->createDecentralizedConstraintsSerial(vf,state,env,con,var,index,t);
        
        default:
            throw sdm::exception::Exception("DecentralizedLP::createDecentralizedConstraints This state is not defined for this function");
            break;
        }
    }

    std::shared_ptr<Action> DecentralizedLP::getVariableResult(const std::shared_ptr<ValueFunction>&vf,const std::shared_ptr<State> &state,const IloCplex &cplex, const IloNumVarArray &var, number t)
    {
        //Specialisation for type of state
        switch (state->getTypeState())
        {
        case TypeState::OCCUPANCY_STATE :
            return this->getVariableResultOccupancy(vf,state,cplex,var,t);
            break;
        case TypeState::SERIAL_OCCUPANCY_STATE :
            return this->getVariableResultSerial(vf,state,cplex,var,t);

        default:
            throw sdm::exception::Exception("DecentralizedLP::getVariableResult This state is not defined for this function");
            break;
        }
    }

    void DecentralizedLP::createDecentralizedVariablesSerial(const std::shared_ptr<ValueFunction>&vf,const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t)
    {
        //Determine the Agent
        auto under_pb = std::dynamic_pointer_cast<SerializedMMDP>(this->world_->getUnderlyingProblem());
        number agent_id = under_pb->getAgentId(t);

        //Create Individual Decentralized Variable
        this->createDecentralizedVariablesIndividual(vf,state, env, var, index, t,agent_id);
    }
    
    void DecentralizedLP::createDecentralizedVariablesOccupancy(const std::shared_ptr<ValueFunction>&vf,const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t)
    {
        auto under_pb = this->world_->getUnderlyingProblem();

        //Create Joint Decentralized Variable
        this->createDecentralizedVariablesJoint(vf,state, env, var, index, t);

        for (auto agent = 0; agent < under_pb->getNumAgents(); ++agent)
        {
            //Create Individual Decentralized Variable
            this->createDecentralizedVariablesIndividual(vf,state, env, var, index, t,agent);
        }
    }

    void DecentralizedLP::createDecentralizedConstraintsSerial(const std::shared_ptr<ValueFunction>&vf,const std::shared_ptr<State>& state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        //Determine the Agent
        auto under_pb = std::dynamic_pointer_cast<SerializedMMDP>(this->world_->getUnderlyingProblem());
        number agent_id = under_pb->getAgentId(t);

        //Create Individual Decentralized Constraints
        this->createDecentralizedConstraintsIndividual(vf,state, env, con, var, index, t,agent_id);
    }

    void DecentralizedLP::createDecentralizedConstraintsOccupancy(const std::shared_ptr<ValueFunction>&vf,const std::shared_ptr<State>& state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        auto under_pb = this->world_->getUnderlyingProblem();
        
        //Create Joint Decentralized Constraints and Control Constraints
        this->createDecentralizedConstraintsJoint(vf,state, env, con, var, index, t);
        this->createDecentralizedControlConstraints(vf,state, env, con, var, index, t);

        for (number agent = 0; agent < under_pb->getNumAgents(); ++agent)
        {
            //Create Individual Decentralized Constraints
            this->createDecentralizedConstraintsIndividual(vf,state, env, con, var, index, t,agent);
        }
    }
    
    std::shared_ptr<Action> DecentralizedLP::getVariableResultSerial(const std::shared_ptr<ValueFunction>&vf,const std::shared_ptr<State> &state,const IloCplex &cplex, const IloNumVarArray &var, number t)
    {
        //Determine the Agent
        auto under_pb = std::dynamic_pointer_cast<SerializedMMDP>(this->world_->getUnderlyingProblem());
        number agent_id = under_pb->getAgentId(t);

        //Determine the element useful for create a DeterminiticDecisionRule
        auto action_and_history_individual = this->getVariableResultIndividual(vf,state,cplex,var,t,agent_id);
        
        //Create the DeterminiticDecisionRule
        return std::make_shared<DeterministicDecisionRule>(action_and_history_individual.second,action_and_history_individual.first);
    }

    std::shared_ptr<Action> DecentralizedLP::getVariableResultOccupancy(const std::shared_ptr<ValueFunction>&vf,const std::shared_ptr<State> &state,const IloCplex &cplex, const IloNumVarArray &var, number t)
    {
        std::vector<std::vector<std::shared_ptr<Item>>> actions;
        std::vector<std::vector<std::shared_ptr<Item>>> joint_histories;

        auto under_pb = this->world_->getUnderlyingProblem();

        //Determine the element useful for create a JointDeterminiticDecisionRule
        for (number agent = 0; agent < under_pb->getNumAgents(); agent++)
        {
            auto action_and_history_individual = this->getVariableResultIndividual(vf,state,cplex,var,t,agent);

            actions.push_back(action_and_history_individual.first);
            joint_histories.push_back(action_and_history_individual.second);
        }
        //Create the JointDeterminiticDecisionRule
        return std::make_shared<JointDeterministicDecisionRule>(joint_histories,actions);
    }
}