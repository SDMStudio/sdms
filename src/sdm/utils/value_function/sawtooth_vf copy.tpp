#include <sdm/utils/value_function/tabular_value_function.hpp>

namespace sdm
{

    template <typename TVector, typename TAction, typename TValue = double>
    TAction MaxPlanValueFunctionLP::greedyMaxPlane(const TVector &occupancy_state, const TVector &hyperplan, double&value, double precision)
    {
        //Identifiant, compteur des contraintes
        number c = 0;

        //<! tracking variables
        std::string VarName;

        TAction a; // Règle de décision maximale, initialisation 
        
        //HistoryTree_p<TObservation> ( trouvé le privé)

        //std::unordered_map<agent, std::unordered_set<HistoryTree_p*>> ihs;

        auto ihs_ = occupancy_state.getAllIndividualHistories();

        // for(agent ag=0; ag<this->getWorld()->getNumAgents(); ++ag){
        //     std::unordered_set<HistoryTree_p> empty;
        //     ihs.emplace(ag, empty);
        // }

        IloEnv env;
        try{
        IloModel model(env);

        // Init the model
        IloRangeArray con(env);
        IloNumVarArray var(env);

        IloObjective obj = IloMaximize(env);

        ///////  BEGIN CORE CPLEX Code  ///////

        // 0. Build variables a(u|o), a_i(u_i|o_i)
        this->setGreedyVariables(occupancy_state, ihs, env, var); // 

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
            a = this->getDecentralizedVariables(cplex, var, o);
        }
        }catch(IloException& e){
        std::cerr << "Concert exception caught: " << e << std::endl;
        }catch (const std::exception &exc){
        // catch anything thrown within try block that derives from std::exception
        std::cerr << "Non-Concert exception caught: " << exc.what() << std::endl;
        }

        env.end();

        return a;

    }

    template <typename TVector, typename TAction, typename TValue = double>
    void MaxPlanValueFunctionLP<TVector, TAction, TValue>::setGreedyVariables(const TVector& o, std::vector<std::set<typename jhistory_type::element_type::ihistory_type>>& ihs, IloEnv& env, IloNumVarArray& var) const
    {
        number index = 0;

        //<! tracking variables
        std::string VarName;

        common::clearVariables();

        //<! 0.b Build variables v_size = objective variable!
        VarName = common::getVarNameWeight(0);
        var.add(IloNumVar(env, -IloInfinity, +IloInfinity, VarName.c_str()));
        common::setNumber(VarName, index++);

        this->setDecentralizedVariables(o, ihs, env, var, index);
  }

  template<typename function, typename ihistory, typename jhistory>
  void lp_maxplane<function, ihistory, jhistory>::setGreedyObjective(const std::shared_ptr<occupancy_map<jhistory>>& o, IloNumVarArray& var, IloObjective& obj, const std::shared_ptr<function>& alpha) const{
    
    number recover = 0;
    horizon h = o->getHorizon();
    double weight = 0.0, factor = 0.0;
    for(auto it=o->begin(); it!=o->end(); ++it){
      auto jh = it->first;
      for(action u=0; u<this->getWorld()->getNumActions(); ++u){
        weight = 0.0;
        for(state x=0; x<this->getWorld()->getNumStates(); ++x){
          //<! 1.a compute factor
          factor = this->getWorld()->getReward(x,u);

          if( h < this->getWorld()->getPlanningHorizon() - 1 ){
            for(observation z_=0; z_<this->getWorld()->getNumObservations(); ++z_){
              auto jh_ = jh->expand(z_);
              for(state x_=0; x_<this->getWorld()->getNumStates(); ++x_){
                factor += this->getWorld()->getDiscount() * this->getWorld()->getDynamics(x,u,z_,x_) * alpha->getValueAt(jh_,x_);
              }
            }
          }

          weight += o->get(x,jh) * factor;
        } // for all x

        //<! 1.b get variable a(u|o)
        recover = common::getNumber(common::getVarNameJointHistoryDecisionRule(u, jh));
        //<! 1.c set coefficient of variable a(u|o) i.e., s(x,o)  [ r(x,u) + \gamma \sum_{x_,z_} P(x_,z_|x,u) * \alpha_i(x_,o_)  ]
        obj.setLinearCoef(var[recover], weight);
      } // for all u
    } // for all o
  }


  // common.hpp à voir pour comprendre








    // template <typename TState, typename TAction, typename TValue>
    // TValue CPLEX<TState, TAction, TValue>::getMaxAt(const TState &serialized_occupancy_state, number t)
    // {
    //     assert(this->getInitFunction() != nullptr); 

    //     TValue argmax_value = -std::numeric_limits<double>::max() ;
    //     TState argmin_;

    //     auto soMDP = std::static_pointer_cast<SerializedOccupancyMDP<TVector, TAction>>(this->getWorld());
    //     auto under_pb = this->getWorld()->getUnderlyingProblem();

    //     number ag_id = state.getCurrentAgentId();

    //     for(const auto &action : under_pb->getActionSpaceAt(ag_id)->getAll())
    //     {

    //         IloEnv env;
    //         try
    //         {   
    //             IloModel model(env);
    //             IloNumVarArray vars(env);
    //             vars.add(IloNumVar(env, 0.0, 40.0));
    //             vars.add(IloNumVar(env));
    //             vars.add(IloNumVar(env));
    //             model.add(IloMaximize(env, vars[0] + 2 * vars[1] + 3 * vars[2]));
    //             model.add(-vars[0] + vars[1] + vars[2] <= 20);
    //             model.add(vars[0] - 3 * vars[1] + vars[2] <= 30);
    //             IloCplex cplex(model);
    //             if (!cplex.solve())
    //             {
    //                 env.error() << "Failed to optimize LP." << endl;
    //                 throw(-1);
    //             }
    //             IloNumArray vals(env);
    //             // env.out() << "Solution status = " << cplex.getStatus() << endl;
    //             // env.out() << "Solution value = " << cplex.getObjValue() << endl;
    //             cplex.getValues(vals, vars);
    //             // env.out() << "Values = " << vals << endl;


    //             if (argmax_value < tmp = cplex.getObjValue())
    //             {
    //                 argmax_value = tmp;
    //                 v_max = v;
    //             }

    //         }
    //         catch (IloException &e)
    //         {
    //             cerr << "Concert exception caught: " << e << endl;
    //         }
    //         catch (...)
    //         {
    //             cerr << "Unknown exception caught" << endl;
    //         }
    //         env.end();
    //     }


    //     // double v_ub_state =this->getInitFunction()->operator()(state, t);
    //     // double min_ext = 0;

    //     // TState argmin_ = state;
    //     // for (const TState &ostate : this->getSupport(t))
    //     // {
    //     //     double v_kappa = this->getValueAt(ostate, t); // Problème boucle infinie 
    //     //     double v_ub_kappa = this->getInitFunction()->operator()(ostate, t);
    //     //     double phi = std::numeric_limits<double>::max();
    //     //     for (auto &x : ostate)
    //     //     {
    //     //         double v_int = (state.at(x.first) / x.second);
    //     //         if (v_int < phi)
    //     //         {
    //     //             phi = v_int;
    //     //         }
    //     //     }
    //     //     double min_int = phi * (v_kappa - v_ub_kappa);
    //     //     if (min_int < min_ext)
    //     //     {
    //     //         min_ext = min_int;
    //     //         argmin_ = ostate; // to verify
    //     //     }
    //     // }
    //     // return std::make_pair(v_ub_state + min_ext, argmin_);
    // }

    template <typename TState, typename TAction, typename TValue>
    void CPLEX<TState, TAction, TValue>::prune(number t)
    {
        std::vector<TState> to_delete;
        
        for (auto iter = this->representation[t].begin(); iter != this->representation[t].end(); iter++)
        {          
            if (this->is_dominated(iter->first, iter->second, t))
            {
                to_delete.push_back(iter->first);
            }
        }

        for(const auto &i : to_delete)
        {
            this->representation[t].erase(i);
        }
    }

    template <typename TState, typename TAction, typename TValue>
    bool CPLEX<TState, TAction, TValue>::is_dominated(const TState &ostate, double value, number t)
    {
        auto pair_witness_ostate = this->getMaxAt(ostate, t);

        if (pair_witness_ostate.second == ostate)
        {
            return false;
        }
        else
        {
            return (pair_witness_ostate.first <= value);
        }
    }
}