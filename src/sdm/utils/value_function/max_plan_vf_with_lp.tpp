#include <sdm/utils/value_function/tabular_value_function.hpp>

namespace sdm
{

  template <typename TVector, typename TAction, typename TValue>
  MaxPlanValueFunctionLP<TVector, TAction, TValue>::MaxPlanValueFunctionLP() {}

  template <typename TVector, typename TAction, typename TValue>
  MaxPlanValueFunctionLP<TVector, TAction, TValue>::MaxPlanValueFunctionLP(std::shared_ptr<SolvableByHSVI<TVector, TAction>> problem, int horizon, std::shared_ptr<Initializer<TVector, TAction>> initializer)
      : MaxPlanValueFunction<TVector, TAction, TValue>(problem, horizon,initializer){}

  template <typename TVector, typename TAction, typename TValue>
  MaxPlanValueFunctionLP<TVector, TAction, TValue>::MaxPlanValueFunctionLP(std::shared_ptr<SolvableByHSVI<TVector, TAction>> problem, int horizon, TValue default_value) : MaxPlanValueFunctionLP(problem, horizon, std::make_shared<ValueInitializer<TVector, TAction>>(default_value)){}

  template <typename TVector, typename TAction, typename TValue >
  TAction MaxPlanValueFunctionLP<TVector, TAction, TValue>::greedyMaxPlane(const TVector &occupancy_state, const TVector &hyperplan, double &value, double)
  {
      auto this->getWorld()->getUnderlyingProblem() = this->getWorld()->getUnderlyingProblem();
      
      //Identifiant, compteur des contraintes
      number c = 0;

      //<! tracking variables
      std::string VarName;

      //<! greedy decision, initialization
      TAction a; 

      //<! set of individual histories per agent
      std::unordered_map<agent, std::unordered_set<typename TVector::jhistory_type::element_type::ihistory_type>> ihs;

      //<! initialization of the above set of individual histories per agent
      for(agent ag=0; ag<this->getWorld()->getUnderlyingProblem()->getNumAgents(); ++ag)
      {
          std::unordered_set<typename TVector::jhistory_type::element_type::ihistory_type> empty;
          ihs.emplace(ag, empty);
      }

      IloEnv env;
      try{
        IloModel model(env);

        // Init the model
        IloRangeArray con(env);
        IloNumVarArray var(env);

        IloObjective obj = IloMaximize(env);

        ///////  BEGIN CORE CPLEX Code  ///////

        // 0. Build variables a(u|o), a_i(u_i|o_i)
        this->setGreedyVariables(occupancy_state, ihs, env, var); 

        // 1. Build objective function \sum_{o,u} A(u|o) \sum_x s(x,o)  [ r(x,u) + \gamma \sum_{x_,z_} P(x_,z_|x,u) * \alpha_i(x_,o_) ]
        this->setGreedyObjective(occupancy_state, var, obj, hyperplan);

        // 3. Build decentralized control constraints [  a(u|o) >= \sum_i a_i(u_i|o_i) + 1 - n ] ---- and ---- [ a(u|o) <= a_i(u_i|o_i) ]
        this->setDecentralizedConstraints(occupancy_state, ihs, env, con, var, c);

        ///////  END CORE  CPLEX Code ///////
        model.add(obj);
        model.add(con);
        IloCplex cplex(model);
        cplex.setOut(env.getNullStream());
        cplex.setWarning(env.getNullStream());

        // Optimize the problem and obtain solution
        if( !cplex.solve() ){
            env.error() << "Failed to optimize MILP" << std::endl;
            // cplex.exportModel("lb_bellman_op.lp");
            // system("cat lb_bellman_op.lp");
            // throw(-1);
        }
        else {
            value = cplex.getObjValue();
            a = this->getDecentralizedVariables(cplex, var, occupancy_state);
        }
      }catch(IloException& e){
          //"Concert exception caught: " << e << std::endl;
      }catch (const std::exception &exc){
        // catch anything thrown within try block that derives from std::exception
        std::cerr << "Non-Concert exception caught: " << exc.what() << std::endl;
      }

      env.end();

      return a;
  }



  template <typename TVector, typename TAction, typename TValue >
  void MaxPlanValueFunctionLP<TVector, TAction, TValue>::setGreedyVariables(const TVector& occupancy_state, std::unordered_map<agent, std::unordered_set<typename TVector::jhistory_type::element_type::ihistory_type>>& ihs, IloEnv& env, IloNumVarArray& var)
  {
    //<! counter for constraints
    number index = 0;

    //<! tracking variables
    std::string VarName;

    // re-init the set of variable names 
    this->variables.clear();

    //<! Build variables v_size = objective variable!
    VarName = this->getVarNameWeight(0);
    var.add(IloNumVar(env, -IloInfinity, +IloInfinity, VarName.c_str()));
    this->setNumber(VarName, index++);

    //<! set decentralized decision rule variables 
    this->setDecentralizedVariables(occupancy_state, ihs, env, var, index);
  }




  template <typename TVector, typename TAction, typename TValue>
  void MaxPlanValueFunctionLP<TVector, TAction, TValue>::setGreedyObjective(const TVector& occupancy_state, IloNumVarArray& var, IloObjective& obj, const TVector& alpha)
  {
    number recover = 0;

    double weight = 0.0, factor = 0.0;

    for(auto it=occupancy_state.begin(); it!=occupancy_state.end(); ++it)
    {
      auto joint_history = occupancy_state.getHistory(it->first);

      for(auto u=0; u<this->getWorld()->getUnderlyingProblem()->getActionSpace()->getNumItems(); ++u)
      {
        weight = 0.0;

        for(auto x=0; x<this->getWorld()->getUnderlyingProblem()->getStateSpace()->getNumItems(); ++x)
        {
          //<! 1.a compute factor
          factor = this->getWorld()->getUnderlyingProblem()->getReward()->getReward(x,u);

          if( occupancy_state.getHorizon() < this->getWorld()->getUnderlyingProblem()->getPlanningHorizon() - 1 ){

            for(auto z_=0; z_<this->getWorld()->getUnderlyingProblem()->getObsSpace()->getNumItems(); ++z_)
            {
              auto joint_history_next = joint_history->expand(this->getWorld()->getUnderlyingProblem()->getObsSpace()->getJointItem(z_));

              for(auto x_=0; x_<this->getWorld()->getUnderlyingProblem()->getStateSpace()->getNumItems(); ++x_)
              {
                factor += this->getWorld()->getUnderlyingProblem()->getDiscount() * this->getWorld()->getUnderlyingProblem()->getObsDynamics()->getDynamics(x,u,z_,x_) * alpha.at(std::make_pair(x_,joint_history_next));
              }
            }
          }

          weight += occupancy_state.at(std::make_pair(x,joint_history)) * factor;
        } // for all x

      //<! 1.b get variable a(u|o)
      recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(u, joint_history));
      //<! 1.c set coefficient of variable a(u|o) i.e., s(x,o)  [ r(x,u) + \gamma \sum_{x_,z_} P(x_,z_|x,u) * \alpha_i(x_,o_)  ]
      obj.setLinearCoef(var[recover], weight);
      } // for all u
    } // for all o
  }




  /**
   * @brief 
   * @tparam TVector 
   * @tparam TAction 
   * @tparam TValue 
   * @param const TVector &  the occupancy state 
   * @param number           the horizon
   * @return TVector 
   */
  template <typename TVector, typename TAction, typename TValue>
  TVector MaxPlanValueFunctionLP<TVector, TAction, TValue>::backup_operator(const TVector &occupancy_state, number t)
  {
    TVector next_hyperplan, new_hyperplan;
    TAction max_decision_rule, joint_decision_rule;
    double max = -std::numeric_limits<double>::max(), value, cvalue;

    for(const auto& hyperplan : this->getSupport(t+1)) // Changer nom de fonction 
    {
      joint_decision_rule = this->greedyMaxPlane(occupancy_state, hyperplan, value, 0);
      if( value > max )
      {
        max_decision_rule = joint_decision_rule;
        next_hyperplan = hyperplan;
        max = value;
      }
    }

    for(auto uncompressed_s_o: occupancy_state.getFullUncompressedOccupancyState())
    {
      auto uncompressed_hidden_state = uncompressed_s_o.first.first; 
      auto uncompressed_joint_history = uncompressed_s_o.first.second; 
      auto compressed_joint_history = occupancy_state.getCompressedJointHistory(uncompressed_joint_history); 
      auto action = max_decision_rule(compressed_joint_history.getIndividualHistories()); 
      
      new_hyperplan[uncompressed_s_o] = this->getWorld()->getUnderlyingProblem()->getReward()->getReward(uncompressed_hidden_state, action);
      for(auto next_hidden_state : this->getWorld()->getReacheableStates(uncompressed_hidden_state, action))
      {
        for(auto next_observation : this->getWorld()->getReacheableObservations(action, next_hidden_state))
        {
          auto next_joint_history =  compressed_joint_history->expand(next_observation);
          new_hyperplan[uncompressed_s_o] += this->getWorld()->getUnderlyingProblem()->getDiscount() * this->getWorld()->getUnderlyingProblem()->getObsDynamics()->getDynamics(uncompressed_hidden_state, action, next_observation, next_hidden_state) * next_hyperplan.at({next_hidden_state,next_joint_history});
        }
      }
    }
    return new_hyperplan;
  }


  template <typename TVector, typename TAction, typename TValue>
  void MaxPlanValueFunctionLP<TVector, TAction, TValue>::updateValueAt(const TVector &state, number t)
  {
    auto new_hyperplan = this->backup_operator<TVector>(state, t);
    this->representation[this->isInfiniteHorizon() ? 0 : t].push_back(new_hyperplan);
    //this->prune(t);
  }

}