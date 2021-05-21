
namespace sdm
{
  template <typename TVector, typename TAction, typename TValue>
  DecentralizedConstraintsLP<TVector, TAction, TValue>::DecentralizedConstraintsLP(std::shared_ptr<SolvableByHSVI<TVector, TAction>> problem) : world_(problem)
  {
  }

  template <typename TVector, typename TAction, typename TValue>
  template <typename T, std::enable_if_t<std::is_any<T, OccupancyState<>, OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>>::value, int>>
  TAction DecentralizedConstraintsLP<TVector, TAction, TValue>::getDecentralizedVariables(const IloCplex &cplex, const IloNumVarArray &var, const TVector &occupancy_state, number)
  {
    number index = 0;
    std::vector<std::vector<typename TAction::output>> actions;
    std::vector<std::vector<typename TAction::input_type>> joint_histories;

    for (number agent = 0; agent < this->world_->getUnderlyingProblem()->getNumAgents(); agent++)
    {
      actions.push_back({});
      joint_histories.push_back({});

      for(const auto& ihistory : occupancy_state.getIndividualHistories(agent))
      {
        joint_histories[agent].push_back(ihistory);

        for(const auto& action : this->world_->getUnderlyingProblem()->getActionSpace()->getSpace(agent)->getAll())
        {
          index = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(action, ihistory, agent));

          //<! https://stackoverflow.com/questions/35904947/problems-to-parse-from-iloboolvararray-to-bool-in-cplex 
          if( cplex.getValue(var[index]) + .5 >= 1 )
          {
            actions[agent].push_back(action);
          }
        }
      }
    }
    return TAction(joint_histories, actions);
  }

  template <typename TVector, typename TAction, typename TValue>
  template <typename T, std::enable_if_t<std::is_any<T, OccupancyState<>, OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>>::value, int>>
  void DecentralizedConstraintsLP<TVector, TAction, TValue>::setDecentralizedVariables(const TVector &occupancy_state, IloEnv &env, IloNumVarArray &var, number &index, number)
  {
    //<! tracking variables
    std::string VarName;

    //<! 0.a Build variables a(u|o), a_i(u_i|o_i)
    for (const auto joint_history : occupancy_state.getJointHistories())
    {
      for (auto action : this->world_->getUnderlyingProblem()->getActionSpace()->getAll())
      {
        //< 0.b Build variables a(u|o)
        VarName = this->getVarNameJointHistoryDecisionRule(action, joint_history);
        var.add(IloNumVar(env, 0.0, +IloInfinity, VarName.c_str()));
        this->setNumber(VarName, index++);
      }
    }

    for (auto agent = 0; agent < this->world_->getUnderlyingProblem()->getNumAgents(); ++agent)
    {
      for(const auto& ihistory : occupancy_state.getIndividualHistories(agent))
      {
        for (auto action : this->world_->getUnderlyingProblem()->getActionSpace()->getSpace(agent)->getAll())
        {
          //<! 0.c Build variables a_i(u_i|o_i)
          VarName = this->getVarNameIndividualHistoryDecisionRule(action, ihistory, agent);
          var.add(IloBoolVar(env, VarName.c_str()));
          this->setNumber(VarName, index++);
        }
      }
    }
  }

  template <typename TVector, typename TAction, typename TValue>
  template <typename T, std::enable_if_t<std::is_any<T, OccupancyState<>, OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>>::value, int>>
  void DecentralizedConstraintsLP<TVector, TAction, TValue>::setDecentralizedConstraints(const TVector &occupancy_state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &c, number)
  {
    number recover;

    for (const auto& jhistory : occupancy_state.getJointHistories())
    {
      for (const auto& action : this->world_->getUnderlyingProblem()->getActionSpace()->getAll())
      {
        //<! 3.a set constraint a(u|o) >= \sum_i a_i(u_i|o_i) + 1 - n
        con.add(IloRange(env, 1 - this->world_->getUnderlyingProblem()->getNumAgents(), +IloInfinity));
        //<! 3.a.1 get variable a(u|o)
        recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action, jhistory));
        //<! 3.a.2 set coefficient of variable a(u|o)
        con[c].setLinearCoef(var[recover], +1.0);
        for (number agent = 0; agent < this->world_->getUnderlyingProblem()->getNumAgents(); ++agent)
        {
          //<! 3.a.3 get variables a_i(u_i|o_i)
          recover = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(action.get(agent), jhistory->getIndividualHistory(agent), agent));
          //<! 3.a.4 set coefficient of variable a_i(u_i|o_i)
          con[c].setLinearCoef(var[recover], -1.0);
        } // for all agent
        //<! increment constraints
        c++;
      } // for all u
    }   // for all o

    // 3.bis Build decentralized control constraints [ a(u|o) <= a_i(u_i|o_i) ]
    for (const auto& jhistory : occupancy_state.getJointHistories())
    {
      for (const auto& action : this->world_->getUnderlyingProblem()->getActionSpace()->getAll())
      {
        for (number agent = 0; agent < this->world_->getUnderlyingProblem()->getNumAgents(); ++agent)
        {
          //<! 3.b set constraint a(u|o) <= a_i(u_i|o_i)
          con.add(IloRange(env, -IloInfinity, 0.0));
          //<! 3.b.1 get variable a(u|o)
          recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action, jhistory));
          //<! 3.b.2 set coefficient of variable a(u|o)
          con[c].setLinearCoef(var[recover], +1.0);
          //<! 3.b.3 get variable a_i(u_i|o_i)
          recover = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(action.get(agent), jhistory->getIndividualHistory(agent), agent));
          //<! 3.b.4 set coefficient of variable a_i(u_i|o_i)
          con[c].setLinearCoef(var[recover], -1.0);
          //<! increment constraints
          c++;
        } // for all agent
      }   // for all u
    }     // for all o

    // 4. Build deterministic policy constraints
    for (number agent = 0; agent < this->world_->getUnderlyingProblem()->getNumAgents(); ++agent)
    {
      for(const auto& ihistory : occupancy_state.getIndividualHistories(agent))
      {
        //<! 4.a set constraint  \sum_{u_i} a_i(u_i|o_i) = 1
        con.add(IloRange(env, 1.0, 1.0));
        for (const auto& iaction : this->world_->getUnderlyingProblem()->getActionSpace()->getSpace(agent)->getAll())
        {
          recover = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(iaction, ihistory, agent));
          con[c].setLinearCoef(var[recover], +1.0);
        }
        //<! increment constraints
        c++;
      }
    }
  }

  template <typename TVector, typename TAction, typename TValue>
  template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int>>
  TAction DecentralizedConstraintsLP<TVector, TAction, TValue>::getDecentralizedVariables(const IloCplex &cplex, const IloNumVarArray &var, const TVector &serial_occupancy_state, number t)
  {
    number index = 0;
    number agent_id = serial_occupancy_state.getCurrentAgentId();

    std::vector<typename TAction::input_type> indiv_histories;
    std::vector<typename TAction::output> actions;

    for (const auto indiv_history : serial_occupancy_state.getIndividualHistories(agent_id))
    {
      for (auto serial_action : this->world_->getUnderlyingProblem()->getActionSpace(t)->getAll())
      {
        index = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(serial_action, indiv_history, agent_id));

        //<! https://stackoverflow.com/questions/35904947/problems-to-parse-from-iloboolvararray-to-bool-in-cplex 
        if (cplex.getValue(var[index]) + .5 >= 1)
        {
          indiv_histories.push_back(indiv_history);
          actions.push_back(serial_action);
        }
      }
    }
    return TAction(indiv_histories, actions);
  }

  template <typename TVector, typename TAction, typename TValue>
  template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int>>
  void DecentralizedConstraintsLP<TVector, TAction, TValue>::setDecentralizedVariables(const TVector &serial_occupancy_state, IloEnv &env, IloNumVarArray &var, number &index, number t)
  {
    //<! tracking variables
    std::string VarName;
    number agent_id = serial_occupancy_state.getCurrentAgentId();

    //<! 0.a Build variables  a_i(u_i|o_i)
    for (const auto indiv_history : serial_occupancy_state.getIndividualHistories(agent_id))
    {
      for (auto serial_action : this->world_->getUnderlyingProblem()->getActionSpace(t)->getAll())
      {
        //<! 0.c Build variables a_i(u_i|o_i)
        VarName = this->getVarNameIndividualHistoryDecisionRule(serial_action, indiv_history, agent_id);
        var.add(IloBoolVar(env, VarName.c_str()));
        this->setNumber(VarName, index++);
      }
    }
  }

  template <typename TVector, typename TAction, typename TValue>
  template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int>>
  void DecentralizedConstraintsLP<TVector, TAction, TValue>::setDecentralizedConstraints(const TVector &serial_occupancy_state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &c, number t)
  {
    number recover = 0;
    number agent_id = serial_occupancy_state.getCurrentAgentId();

    // 4. Build deterministic policy constraints
    for (const auto indiv_history : serial_occupancy_state.getIndividualHistories(agent_id))
    {
      //<! 4.a set constraint  \sum_{u_i} a_i(u_i|o_i) = 1
      con.add(IloRange(env, 1.0, 1.0));
      for (action serial_action : this->world_->getUnderlyingProblem()->getActionSpace(t)->getAll())
      {
        recover = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(serial_action, indiv_history, agent_id));
        con[c].setLinearCoef(var[recover], +1.0);
      }
      //<! increment constraints
      c++;
    }
  }

}