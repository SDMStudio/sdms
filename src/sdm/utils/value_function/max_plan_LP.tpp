#include <sdm/utils/value_function/tabular_value_function.hpp>

namespace sdm
{

  template <typename TVector, typename TAction, typename TValue>
  MaxPlanValueFunctionLP<TVector, TAction, TValue>::MaxPlanValueFunctionLP() {}

  template <typename TVector, typename TAction, typename TValue>
  MaxPlanValueFunctionLP<TVector, TAction, TValue>::MaxPlanValueFunctionLP(std::shared_ptr<SolvableByHSVI<TVector, TAction>> problem, int horizon, std::shared_ptr<Initializer<TVector, TAction>> initializer)
      : MaxPlanValueFunction<TVector, TAction, TValue>(problem, horizon,initializer)
  {
  }

  template <typename TVector, typename TAction, typename TValue>
  MaxPlanValueFunctionLP<TVector, TAction, TValue>::MaxPlanValueFunctionLP(std::shared_ptr<SolvableByHSVI<TVector, TAction>> problem, int horizon, TValue default_value) : MaxPlanValueFunctionLP(problem, horizon, std::make_shared<ValueInitializer<TVector, TAction>>(default_value))
  {
  }
  // A quoi sert value ? Precision ? 
  template <typename TVector, typename TAction, typename TValue >
  TAction MaxPlanValueFunctionLP<TVector, TAction, TValue>::greedyMaxPlane(const TVector &occupancy_state, const TVector &hyperplan, double&value, double)
  {
      std::cout<<"\n GreedyMaxPlane";

      auto under_problem = this->getWorld()->getUnderlyingProblem();
      //Identifiant, compteur des contraintes
      number c = 0;

      //<! tracking variables
      std::string VarName;

      TAction a; // Règle de décision maximale, initialisation 

      std::unordered_map<agent, std::unordered_set<typename TVector::jhistory_type::element_type::ihistory_type>> ihs;

      for(agent ag=0; ag<under_problem->getNumAgents(); ++ag){
          std::unordered_set<typename TVector::jhistory_type::element_type::ihistory_type> empty;
          ihs.emplace(ag, empty);
      }
      //On créer un ensemble vide pour chaque agent, forcement ensemble vide ? Car aussi non, on a cette fonction       
      //auto ihs_ = occupancy_state.getAllIndividualHistories(); qui permet de donner les historiques individuels pour chaque agent (un vect au lieu d'une map)

      std::cout<<"\n Before CPLEX";

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
    std::cout<<"\n SetGreedyVariables";
    number index = 0;

    //<! tracking variables
    std::string VarName;

    this->variables.clear();

    //<! 0.b Build variables v_size = objective variable!
    VarName = this->getVarNameWeight(0);
    var.add(IloNumVar(env, -IloInfinity, +IloInfinity, VarName.c_str()));
    this->setNumber(VarName, index++);

    this->setDecentralizedVariables(occupancy_state, ihs, env, var, index);
  }

  template <typename TVector, typename TAction, typename TValue>
  void MaxPlanValueFunctionLP<TVector, TAction, TValue>::setGreedyObjective(const TVector& occupancy_state, IloNumVarArray& var, IloObjective& obj, const TVector& alpha)
  {
    std::cout<<"\n SetGreedyObjective";

    auto under_problem = this->getWorld()->getUnderlyingProblem();
 
    number recover = 0;

    horizon h = occupancy_state.getHorizon();

    double weight = 0.0, factor = 0.0;

    for(auto it=occupancy_state.begin(); it!=occupancy_state.end(); ++it)
    {
      auto joint_history = occupancy_state.getHistory(it->first);

      std::cout<<"\n greedy objec history"<<joint_history;

      for(auto u=0; u<under_problem->getActionSpace()->getNumItems(); ++u)
      {
        weight = 0.0;
        for(auto x=0; x<under_problem->getStateSpace()->getNumItems(); ++x)
        {
          //<! 1.a compute factor
          factor = under_problem->getReward()->getReward(x,u);

          if( h < under_problem->getPlanningHorizon() - 1 ){
            for(auto z_=0; z_<under_problem->getObsSpace()->getNumItems(); ++z_)
            {

              auto joint_history_next = joint_history->expand(under_problem->getObsSpace()->getJointItem(z_));

              for(auto x_=0; x_<under_problem->getStateSpace()->getNumItems(); ++x_)
              {
                factor += under_problem->getDiscount() * under_problem->getObsDynamics()->getDynamics(x,u,z_,x_) * alpha.at(std::make_pair(x_,joint_history_next));
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

  template <typename TVector, typename TAction, typename TValue>
  TAction MaxPlanValueFunctionLP<TVector, TAction, TValue>::getDecentralizedVariables(const IloCplex& cplex, const IloNumVarArray& var, const TVector& occupancy_state)
  {
    std::cout<<"\n getDecentralizedVariables";
    auto under_problem = this->getWorld()->getUnderlyingProblem();

    number index = 0;
    TAction a; // Attention à vérifier // std::make_shared<joint_decision_rule<ihistory,jhistory>>();
    
    for(auto iter=occupancy_state.begin(); iter != occupancy_state.end(); ++iter){

      auto joint_history = occupancy_state.getHistory(it->first);

      for(auto u=0; u<under_problem->getActionSpace()->getNumItems(); ++u){

        index = this->getNumber(this->getVarNameJointHistoryDecisionRule(u,joint_history));
        a->setJointProbability(joint_history, u, cplex.getValue(var[index]));
      }
    }

    return a;
  }

  template <typename TVector, typename TAction, typename TValue>
  void MaxPlanValueFunctionLP<TVector, TAction, TValue>::setDecentralizedVariables(const TVector& occupancy_state, std::unordered_map<agent, std::unordered_set<typename TVector::jhistory_type::element_type::ihistory_type>>& ihs, IloEnv& env, IloNumVarArray& var, number& index)
  {

    std::cout<<"\n setDecentralizedVariables";

    //<! tracking variables
    std::string VarName;
    auto under_problem = this->getWorld()->getUnderlyingProblem();

    //<! 0.a Build variables a(u|o), a_i(u_i|o_i)
    for(auto it=occupancy_state.begin(); it!=occupancy_state.end(); ++it)
    {
      auto joint_history = occupancy_state.getHistory(it->first);

      for(auto u=0; u<under_problem->getActionSpace()->getNumItems(); ++u)
      {
        //< 0.b Build variables a(u|o)
        VarName = this->getVarNameJointHistoryDecisionRule(u,joint_history);
        std::cout<<"\n build var Joint "<<VarName;
        var.add(IloNumVar(env, 0.0, +IloInfinity, VarName.c_str()));
        this->setNumber(VarName, index++);
      }

      for(auto ag=0; ag<under_problem->getNumAgents(); ++ag)
      {
        auto ih = joint_history->getIndividualHistory(ag);
        if( ihs[ag].find( ih ) == ihs[ag].end() )
        {
          for(auto iu=0; iu<under_problem->getActionSpace()->getSpace(ag)->getNumItems(); ++iu)
          {
            //<! 0.c Build variables a_i(u_i|o_i)
            VarName = this->getVarNameIndividualHistoryDecisionRule(iu, ih, ag);
            var.add(IloBoolVar(env, 0.0, 1.0, VarName.c_str()));
            this->setNumber(VarName, index++);
          }
          ihs[ag].insert( ih );
        }
      }
    }
  }

  template <typename TVector, typename TAction, typename TValue>
  void MaxPlanValueFunctionLP<TVector, TAction, TValue>::setDecentralizedConstraints(const TVector& occupancy_state, std::unordered_map<agent, std::unordered_set<typename TVector::jhistory_type::element_type::ihistory_type>>& ihs, IloEnv& env, IloRangeArray& con, IloNumVarArray& var, number& c)
  {
    auto under_problem = this->getWorld()->getUnderlyingProblem();

    std::cout<<"\n setDecentralizedConstraints";

    number recover = 0;
    number number_agent = under_problem->getNumAgents();

    for(auto it=occupancy_state.begin(); it!=occupancy_state.end(); ++it)
    {
      auto joint_history = occupancy_state.getHistory(it->first);
      for(auto u=0; u<under_problem->getActionSpace()->getNumItems(); ++u)
      {
        //<! 3.a set constraint a(u|o) >= \sum_i a_i(u_i|o_i) + 1 - n
        con.add(IloRange(env, 1 - number_agent, +IloInfinity));
        //<! 3.a.1 get variable a(u|o)
        recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(u,joint_history));
        //<! 3.a.2 set coefficient of variable a(u|o)
        con[c].setLinearCoef(var[recover], +1.0);
        for(number ag=0; ag<number_agent; ++ag)
        {
          auto ih = joint_history->getIndividualHistory(ag);

          auto iu = under_problem->getActionSpace()->getItem(ag,u);
          //<! 3.a.3 get variables a_i(u_i|o_i)
          recover = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(iu, ih, ag));
          //<! 3.a.4 set coefficient of variable a_i(u_i|o_i)
          con[c].setLinearCoef(var[recover], -1.0);
        } // for all agent
        //<! increment constraints
        c++;
      } // for all u
    } // for all o

    // 3.bis Build decentralized control constraints [ a(u|o) <= a_i(u_i|o_i) ]
    for(auto it=occupancy_state.begin(); it!=occupancy_state.end(); ++it){
      auto jh = occupancy_state.getHistory(it->first);
      for(action u=0; u<under_problem->getActionSpace()->getNumItems(); ++u){
        for(agent ag=0; ag<number_agent; ++ag){
          auto ih = joint_history->getIndividualHistory(ag);
          auto iu = under_problem->getActionSpace()->getItem(ag,u);
          //<! 3.b set constraint a(u|o) <= a_i(u_i|o_i)
          con.add(IloRange(env, -IloInfinity, 0.0));
          //<! 3.b.1 get variable a(u|o)
          recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(u,jh));
          //<! 3.b.2 set coefficient of variable a(u|o)
          con[c].setLinearCoef(var[recover], +1.0);
          //<! 3.b.3 get variable a_i(u_i|o_i)
          recover = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(iu, ih, ag));
          //<! 3.b.4 set coefficient of variable a_i(u_i|o_i)
          con[c].setLinearCoef(var[recover], -1.0);
          //<! increment constraints
          c++;
        } // for all agent
      } // for all u
    } // for all o

    // 4. Build deterministic policy constraints
    for(agent ag=0; ag<number_agent; ++ag){
      for(auto ih : ihs[ag]){
        //<! 4.a set constraint  \sum_{u_i} a_i(u_i|o_i) = 1
        con.add(IloRange(env, 1.0, 1.0));
        for(action iu=0; iu<under_problem->getActionSpace()->getSpace(ag)->getNumItems(); ++iu){
          recover = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(iu, ih, ag));
          con[c].setLinearCoef(var[recover], +1.0);
        }
        //<! increment constraints
        c++;
      }
    }
  }

  template <typename TVector, typename TAction, typename TValue>
  TVector MaxPlanValueFunctionLP<TVector, TAction, TValue>::backup_operator(const TVector &occupancy_state, number t)
  {
    double max = -std::numeric_limits<double>::max(),value;
    TAction max_decision_rule, joint_decision_rule;

    for(auto hyperplan : this->getSupport(t+1)) // Changer nom de fonction 
    {
      joint_decision_rule = this->greedyMaxPlane(occupancy_state, hyperplan, value, 0);
      if( value > max )
      {
        max_decision_rule = joint_decision_rule;
        max = value;
      }
    }

    TVector new_hyperplan;
    for(auto uncompressed_s_o: occupancy_state.getFullUncompressedOccupancyState())
    {
      auto uncompressed_hidden_state = uncompressed_s_o.first.first; 
      auto uncompressed_joint_history = uncompressed_s_o.first.second; 
      auto uncompressed_action = max_decision_rule(occupancy_state.getJointLabels(uncompressed_joint_history.getIndividualHistories())); 
      
      for(auto next_hidden_state : this->getWorld()->getReacheableStates(uncompressed_hidden_state,uncompressed_action))
      {
        for(auto next_observation : this->getWorld()->getReacheableObservations(uncompressed_action,next_hidden_state))
        {

        }
      }
      new_hyperplan[uncompressed_s_o] = ;
    }

  }


  template <typename TVector, typename TAction, typename TValue>
  void MaxPlanValueFunctionLP<TVector, TAction, TValue>::updateValueAt(const TVector &state, number t)
  {

    auto new_hyperplan = this->backup_operator<TVector>(state, t);

    this->representation[this->isInfiniteHorizon() ? 0 : t].push_back(new_hyperplan);

    this->prune(t);
  }

  //*********** Fonction moins importante à partir d'ici

  template <typename TVector, typename TAction, typename TValue>
  std::string MaxPlanValueFunctionLP<TVector, TAction, TValue>::getVarNameJointHistoryDecisionRule(action a, typename TVector::jhistory_type jh)
  {
    std::ostringstream oss;
    oss << "jdr" << "." << a  << "." << jh;
    return oss.str();
  }

  template <typename TVector, typename TAction, typename TValue>
  void MaxPlanValueFunctionLP<TVector, TAction, TValue>::setNumber(const std::string& name, number id)
  {
    if( this->variables.find(name) == this->variables.end())
    {
      this->variables.emplace(name, id);
    }
    else
    {
      this->variables.at(name) = id;
    }
  }

  template <typename TVector, typename TAction, typename TValue>
  std::string MaxPlanValueFunctionLP<TVector, TAction, TValue>::getVarNameWeight(number index)
  {
    std::ostringstream oss;
    oss << "weight" << "." << index;
    return oss.str();
  }

  template <typename TVector, typename TAction, typename TValue>
  std::string MaxPlanValueFunctionLP<TVector, TAction, TValue>::getVarNameIndividualHistoryDecisionRule(action a, typename TVector::jhistory_type::element_type::ihistory_type ih, agent ag)
  {
    std::ostringstream oss;
    oss << "idr" << "." << a  << "." << ih << "." << ag;
    return oss.str();
  }

  template <typename TVector, typename TAction, typename TValue>
  number MaxPlanValueFunctionLP<TVector, TAction, TValue>::getNumber(const std::string& name)
  {
    // assert( common::variables.find(name) != common::variables.end() );
    std::cout<<"\n name :"<<name;

    for(auto a : this->variables)
    {
      std::cout<<"\n this variable name:"<<a.first;
    }

    if( this->variables.find(name) == this->variables.end() )
    {
      std::cerr << "something went wrong in here!"<< "\tname:" << name << std::endl;
      exit(-1);
    }
    return this->variables.at( name );
  }


  // common.hpp à voir pour comprendre
}