#include <sdm/utils/value_function/lp_decentralized.hpp>

namespace sdm{
  template<typename ihistory, typename jhistory>
  std::shared_ptr<Action> lp_decentralized<ihistory,jhistory>::getDecentralizedVariables(const IloCplex& cplex, const IloNumVarArray& var, const std::shared_ptr<occupancy_map<jhistory>>& o){
    number index = 0;
    auto a = std::make_shared<joint_decision_rule<ihistory,jhistory>>();
    for(auto iter=o->begin(); iter != o->end(); ++iter){
      for(action u=0; u<common::model->getNumActions(); ++u){
        index = common::getNumber(common::getVarNameJointHistoryDecisionRule(u, iter->first));
        a->setJointProbability(iter->first, u, cplex.getValue(var[index]));
      }
    }

    return a;
  }

  template<typename ihistory, typename jhistory>
  void lp_decentralized<ihistory,jhistory>::setDecentralizedVariables(const std::shared_ptr<occupancy_map<jhistory>>& o, std::unordered_map<agent, std::unordered_set<ihistory*>>& ihs, IloEnv& env, IloNumVarArray& var, number& index) const{
    //<! tracking variables
    std::string VarName;

    //<! 0.a Build variables a(u|o), a_i(u_i|o_i)
    for(auto it=o->begin(); it!=o->end(); ++it){
      for(action u=0; u<common::model->getNumActions(); ++u){
        //< 0.b Build variables a(u|o)
        VarName = common::getVarNameJointHistoryDecisionRule(u,it->first);
        var.add(IloNumVar(env, 0.0, +IloInfinity, VarName.c_str()));
        common::setNumber(VarName, index++);
      }

      for(agent ag=0; ag<common::model->getNumAgents(); ++ag){
        auto ih = it->first->at(ag);
        if( ihs[ag].find( ih ) == ihs[ag].end() ){
          for(action iu=0; iu<common::model->getNumActions(ag); ++iu){
            //<! 0.c Build variables a_i(u_i|o_i)
            VarName = common::getVarNameIndividualHistoryDecisionRule(iu, ih, ag);
            var.add(IloBoolVar(env, 0.0, 1.0, VarName.c_str()));
            common::setNumber(VarName, index++);
          }
          ihs[ag].insert( ih );
        }
      }
    }
  }

  template<typename ihistory, typename jhistory>
  void lp_decentralized<ihistory,jhistory>::setDecentralizedConstraints(const std::shared_ptr<occupancy_map<jhistory>>& o, std::unordered_map<agent, std::unordered_set<ihistory*>>& ihs, IloEnv& env, IloRangeArray& con, IloNumVarArray& var, number& c) const{
    number recover = 0;

    for(auto it=o->begin(); it!=o->end(); ++it){
      auto jh = it->first;
      for(action u=0; u<common::model->getNumActions(); ++u){
        //<! 3.a set constraint a(u|o) >= \sum_i a_i(u_i|o_i) + 1 - n
        con.add(IloRange(env, 1 - common::model->getNumAgents(), +IloInfinity));
        //<! 3.a.1 get variable a(u|o)
        recover = common::getNumber(common::getVarNameJointHistoryDecisionRule(u,jh));
        //<! 3.a.2 set coefficient of variable a(u|o)
        con[c].setLinearCoef(var[recover], +1.0);
        for(agent ag=0; ag<common::model->getNumAgents(); ++ag){
          auto ih = jh->at(ag);
          auto iu = common::model->getActionIndex(ag,u);
          //<! 3.a.3 get variables a_i(u_i|o_i)
          recover = common::getNumber(common::getVarNameIndividualHistoryDecisionRule(iu, ih, ag));
          //<! 3.a.4 set coefficient of variable a_i(u_i|o_i)
          con[c].setLinearCoef(var[recover], -1.0);
        } // for all agent
        //<! increment constraints
        c++;
      } // for all u
    } // for all o

    // 3.bis Build decentralized control constraints [ a(u|o) <= a_i(u_i|o_i) ]
    for(auto it=o->begin(); it!=o->end(); ++it){
      auto jh = it->first;
      for(action u=0; u<common::model->getNumActions(); ++u){
        for(agent ag=0; ag<common::model->getNumAgents(); ++ag){
          auto ih = jh->at(ag);
          auto iu = common::model->getActionIndex(ag,u);
          //<! 3.b set constraint a(u|o) <= a_i(u_i|o_i)
          con.add(IloRange(env, -IloInfinity, 0.0));
          //<! 3.b.1 get variable a(u|o)
          recover = common::getNumber(common::getVarNameJointHistoryDecisionRule(u,jh));
          //<! 3.b.2 set coefficient of variable a(u|o)
          con[c].setLinearCoef(var[recover], +1.0);
          //<! 3.b.3 get variable a_i(u_i|o_i)
          recover = common::getNumber(common::getVarNameIndividualHistoryDecisionRule(iu, ih, ag));
          //<! 3.b.4 set coefficient of variable a_i(u_i|o_i)
          con[c].setLinearCoef(var[recover], -1.0);
          //<! increment constraints
          c++;
        } // for all agent
      } // for all u
    } // for all o

    // 4. Build deterministic policy constraints
    for(agent ag=0; ag<common::model->getNumAgents(); ++ag){
      for(auto ih : ihs[ag]){
        //<! 4.a set constraint  \sum_{u_i} a_i(u_i|o_i) = 1
        con.add(IloRange(env, 1.0, 1.0));
        for(action iu=0; iu<common::model->getNumActions(ag); ++iu){
          recover = common::getNumber(common::getVarNameIndividualHistoryDecisionRule(iu, ih, ag));
          con[c].setLinearCoef(var[recover], +1.0);
        }
        //<! increment constraints
        c++;
      }
    }
  }

  template class lp_decentralized<observation_ihistory, observation_jhistory>;
}
