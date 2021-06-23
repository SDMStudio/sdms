#include <sdm/utils/value_function/backup/decentralized_constraints_with_lp.hpp>
// #include <sdm/world/base/mpomdp_interface.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>
#include <sdm/world/serialized_mpomdp.hpp>

namespace sdm
{  
  DecentralizedConstraintsLP::DecentralizedConstraintsLP() {}

  DecentralizedConstraintsLP::DecentralizedConstraintsLP(const std::shared_ptr<SolvableByHSVI>& problem) : world_(problem)
  {
  }

  std::shared_ptr<Action> DecentralizedConstraintsLP::getDecentralizedVariables(const IloCplex &cplex, const IloNumVarArray &var, const std::shared_ptr<OccupancyStateInterface> &occupancy_state, number t)
  {
    switch (occupancy_state->getTypeState())
    {
    case TypeState::OCCUPANCY_STATE: 
      return this->getDecentralizedVariablesOccupancy(cplex,var,occupancy_state,t);
      break;
    case TypeState::SERIAL_OCCUPANCY_STATE: 
      return this->getDecentralizedVariablesSerialOccupancy(cplex,var,occupancy_state,t);
      break;
    
    default:
      exit(-1);
      break;
    }
  }

  std::shared_ptr<Action> DecentralizedConstraintsLP::getDecentralizedVariablesOccupancy(const IloCplex &cplex, const IloNumVarArray &var, const std::shared_ptr<OccupancyStateInterface> &occupancy_state, number t)
  {
    number index = 0;
    std::vector<std::vector<std::shared_ptr<Item>>> actions;
    std::vector<std::vector<std::shared_ptr<Item>>> joint_histories;

    auto under_pb = std::dynamic_pointer_cast<MPOMDPInterface>(this->world_->getUnderlyingProblem());

    for (number agent = 0; agent < under_pb->getNumAgents(); agent++)
    {
      actions.push_back({});
      joint_histories.push_back({});

      for(const auto& ihistory : occupancy_state->getIndividualHistories(agent))
      {
        joint_histories[agent].push_back(ihistory);
        for(const auto& action : *under_pb->getActionSpace(agent,t))
        {
          index = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(action->toAction(), ihistory, agent));
          
          if( cplex.getValue(var[index]) + .5 >= 1 )
          {
            actions[agent].push_back(action);
          }
        }
      }
    }

    return std::make_shared<JointDeterministicDecisionRule>(joint_histories,actions);
  }

  std::shared_ptr<Action> DecentralizedConstraintsLP::getDecentralizedVariablesSerialOccupancy(const IloCplex &cplex, const IloNumVarArray &var, const std::shared_ptr<OccupancyStateInterface> &occupancy_state, number t)
  {
    number index = 0;
    std::vector<std::shared_ptr<Item>> actions;
    std::vector<std::shared_ptr<Item>> indiv_histories;

    auto under_pb = std::dynamic_pointer_cast<SerializedMPOMDP>(this->world_->getUnderlyingProblem());
    auto agent = under_pb->getAgentId(t);

    for(const auto& ihistory : occupancy_state->getIndividualHistories(agent))
    {
      indiv_histories.push_back(ihistory);
      for(const auto& action : *under_pb->getActionSpace(agent,t))
      {
        index = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(action->toAction(), ihistory, agent));
        
        if( cplex.getValue(var[index]) + .5 >= 1 )
        {
          actions.push_back(action);
        }
      }
    }
    return std::make_shared<DeterministicDecisionRule>(indiv_histories,actions);
  }

  void DecentralizedConstraintsLP::setDecentralizedVariables(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, IloEnv &env, IloNumVarArray &var, number &index, number t)
  {
    switch (occupancy_state->getTypeState())
    {
    case TypeState::OCCUPANCY_STATE: 
      this->setDecentralizedVariablesOccupancy(occupancy_state,env,var,index,t);
      break;
    case TypeState::SERIAL_OCCUPANCY_STATE: 
      this->setDecentralizedVariablesSerialOccupancy(occupancy_state,env,var,index,t);
      break;
    
    default:
      exit(-1);
      break;
    }
  }

  void DecentralizedConstraintsLP::setDecentralizedVariablesOccupancy(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, IloEnv &env, IloNumVarArray &var, number &index, number t)
  {
    auto under_pb = std::dynamic_pointer_cast<MPOMDPInterface>(this->world_->getUnderlyingProblem());

    //<! tracking variables
    std::string VarName;

    //<! 0.a Build variables a(u|o), a_i(u_i|o_i)
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

    for (auto agent = 0; agent < under_pb->getNumAgents(); ++agent)
    {
      for(const auto& ihistory : occupancy_state->getIndividualHistories(agent))
      {
        for (const auto& action : *under_pb->getActionSpace(agent,t))
        {
          //<! 0.c Build variables a_i(u_i|o_i)
          VarName = this->getVarNameIndividualHistoryDecisionRule(action->toAction(), ihistory, agent);
          var.add(IloBoolVar(env, VarName.c_str()));
          this->setNumber(VarName, index++);
        }
      }
    }
  }

  void DecentralizedConstraintsLP::setDecentralizedVariablesSerialOccupancy(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, IloEnv &env, IloNumVarArray &var, number &index, number t)
  {
    auto under_pb = std::dynamic_pointer_cast<SerializedMPOMDP>(this->world_->getUnderlyingProblem());

    //<! tracking variables
    std::string VarName;
    number agent_id = under_pb->getAgentId(t);

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

  void DecentralizedConstraintsLP::setDecentralizedConstraints(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
  {
    switch (occupancy_state->getTypeState())
    {
    case TypeState::OCCUPANCY_STATE: 
      this->setDecentralizedConstraintsOccupancy(occupancy_state,env,con,var,index,t);
      break;
    case TypeState::SERIAL_OCCUPANCY_STATE: 
      this->setDecentralizedConstraintsSerialOccupancy(occupancy_state,env,con,var,index,t);
      break;
    
    default:
      exit(-1);
      break;
    }
  }

  void DecentralizedConstraintsLP::setDecentralizedConstraintsOccupancy(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
  {
    auto under_pb = std::dynamic_pointer_cast<MPOMDPInterface>(this->world_->getUnderlyingProblem());

    number recover;

    for (const auto& jhistory : occupancy_state->getJointHistories())
    {
      for (const auto& action : *under_pb->getActionSpace(t))
      {
        auto joint_action = std::static_pointer_cast<Joint<std::shared_ptr<Action>>>(action->toAction());

        //<! 3.a set constraint a(u|o) >= \sum_i a_i(u_i|o_i) + 1 - n
        con.add(IloRange(env, 1 - under_pb->getNumAgents(), +IloInfinity));
        //<! 3.a.1 get variable a(u|o)
        recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action->toAction(), jhistory->toJointHistoryTree()));
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

    // 3.bis Build decentralized control constraints [ a(u|o) <= a_i(u_i|o_i) ]
    for (const auto& jhistory : occupancy_state->getJointHistories())
    {
      for (const auto& action : *under_pb->getActionSpace(t))
      {
        auto joint_action = std::static_pointer_cast<Joint<std::shared_ptr<Action>>>(action->toAction());

        for (number agent = 0; agent < under_pb->getNumAgents(); ++agent)
        {
          //<! 3.b set constraint a(u|o) <= a_i(u_i|o_i)
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

    // 4. Build deterministic policy constraints
    for (number agent = 0; agent < under_pb->getNumAgents(); ++agent)
    {
      for(const auto& ihistory : occupancy_state->getIndividualHistories(agent))
      {
        //<! 4.a set constraint  \sum_{u_i} a_i(u_i|o_i) = 1
        con.add(IloRange(env, 1.0, 1.0));
        for (const auto& iaction : *under_pb->getActionSpace(agent,t))
        {
          recover = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(iaction->toAction(), ihistory, agent));
          con[index].setLinearCoef(var[recover], +1.0);
        }
        //<! increment constraints
        index++;
      }
    }

  }

  void DecentralizedConstraintsLP::setDecentralizedConstraintsSerialOccupancy(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
  {
    auto under_pb = std::dynamic_pointer_cast<SerializedMPOMDP>(this->world_->getUnderlyingProblem());
    number agent_id = under_pb->getAgentId(t);

    number recover = 0;

    // 4. Build deterministic policy constraints
    for (const auto& indiv_history : occupancy_state->getIndividualHistories(agent_id))
    {
      //<! 4.a set constraint  \sum_{u_i} a_i(u_i|o_i) = 1
      con.add(IloRange(env, 1.0, 1.0));
      for (const auto& serial_action : *under_pb->getActionSpace(t))
      {
        recover = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(serial_action->toAction(), indiv_history, agent_id));
        con[index].setLinearCoef(var[recover], +1.0);
      }
      //<! increment constraints
      index++;
    }

  }
}