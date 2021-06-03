#include <sdm/utils/value_function/max_plan_vf_with_wcsp.hpp>

namespace sdm
{

  template <typename TVector, typename TAction, typename TValue>
  MaxPlanValueFunctionWCSP<TVector, TAction, TValue>::MaxPlanValueFunctionWCSP() {}

  template <typename TVector, typename TAction, typename TValue>
  MaxPlanValueFunctionWCSP<TVector, TAction, TValue>::MaxPlanValueFunctionWCSP(std::shared_ptr<SolvableByHSVI<TVector, TAction>> problem, int horizon, std::shared_ptr<Initializer<TVector, TAction>> initializer, int freq_prunning)
      : MaxPlanValueFunction<TVector, TAction, TValue>(problem, horizon, initializer,freq_prunning) 
  {}

  template <typename TVector, typename TAction, typename TValue>
  MaxPlanValueFunctionWCSP<TVector, TAction, TValue>::MaxPlanValueFunctionWCSP(std::shared_ptr<SolvableByHSVI<TVector, TAction>> problem, int horizon, TValue default_value, int freq_prunning) : MaxPlanValueFunctionLP(problem, horizon, std::make_shared<ValueInitializer<TVector, TAction>>(default_value), freq_prunning)
  {}

  template <typename TVector, typename TAction, typename TValue = double>
  TAction MaxPlanValueFunctionWCSP<TVector,TAction,TValue>::getGreedyActionAt(const TVector &occupancy_state, double &value)
  {
      // std::unordered_map<std::string, number> vars; // this stores the toulbar2 variables along with their identifiers
      number index;

      tb2init();              // must be call before setting specific ToulBar2 options and creating a model
      ToulBar2::verbose = -1; // change to 0 or higher values to see more trace information

      // create a problem with multiple variables
      std::shared_ptr<WeightedCSPSolver> wcsp_solver = std::shared_ptr<WeightedCSPSolver>(WeightedCSPSolver::makeWeightedCSPSolver(MAX_COST));

      // building variables a^i(u^i|o^i) for each agent i
      for (number agent = 0; agent < this->world_->getUnderlyingProblem()->getNumAgents(); ++agent)
      {
        for(const auto& ihistory : occupancy_state.getIndividualHistories(agent))
        {
          index = wcsp_solver->getWCSP()->makeEnumeratedVariable(getVarNameIndividualHistory(ihistory,agent), 0, this->world_->getUnderlyingProblem()->getActionSpace()->getSpace(agent)->getNumItems() - 1);
          this->variables.emplace(ihistory->str(), index);
          index ++;
        }
      }

      // Creation of the cost network

      // Go over all joint histories 
      for(const auto &joint_history : occupancy_state.getJointHistories())
      {
        //Go over all joint action
        for(const auto &joint_action : this->getWorld()->getUnderlyingProblem()->getActionSpace()->getAll())
        {

          for (number agent = 0; agent < this->world_->getUnderlyingProblem()->getNumAgents(); ++agent)
          {
            

          }
        }
      }




      // building cost network functions
      for (iter = this->s->begin(); iter != this->s->end(); ++iter)
      {
        std::vector<Cost> costs;
        for (u = 0; u < common::model->getNumActions(); u++)
        {
          costs.push_back(this->getCost(iter->second.first * this->getValueAt(iter->first, u)));
        }
        wcsp_solver->getWCSP()->postBinaryConstraint(vars[iter->first->at(0)->to_string()], vars[iter->first->at(1)->to_string()], costs);
      }

  //   agent i;
  //   number index;
  //   action u, u1, u2;
  //   this->setJointTypes(s);
  //   auto iter = this->s->begin();
  //   std::unordered_map<std::string, number> vars; // this stores the toulbar2 variables along with their identifiers
  //   auto a = std::make_shared<joint_decision_rule<i_type, j_type>>();

  //   tb2init();              // must be call before setting specific ToulBar2 options and creating a model
  //   ToulBar2::verbose = -1; // change to 0 or higher values to see more trace information

  //   // create a problem with multiple variables
  //   std::shared_ptr<WeightedCSPSolver> wcsp_solver = std::shared_ptr<WeightedCSPSolver>(WeightedCSPSolver::makeWeightedCSPSolver(MAX_COST));

  //   // building variables a^i(u^i|o^i) for each agent i
  //   for (i = 0; i < common::model->getNumAgents(); ++i)
  //   {
  //     for (auto ih : this->is[i])
  //     {
  //       index = wcsp_solver->getWCSP()->makeEnumeratedVariable(ih.first->to_string(), 0, common::model->getNumActions(i) - 1);
  //       vars.emplace(ih.first->to_string(), index);
  //     }
  //   }

  //   // building cost network functions
  //   for (iter = this->s->begin(); iter != this->s->end(); ++iter)
  //   {
  //     std::vector<Cost> costs;
  //     for (u = 0; u < common::model->getNumActions(); u++)
  //     {
  //       costs.push_back(this->getCost(iter->second.first * this->getValueAt(iter->first, u)));
  //     }
  //     wcsp_solver->getWCSP()->postBinaryConstraint(vars[iter->first->at(0)->to_string()], vars[iter->first->at(1)->to_string()], costs);
  //   }

  //   wcsp_solver->getWCSP()->sortConstraints(); // must be done before the search

  //   if (wcsp_solver->solve())
  //   {
  //     value = 0;
  //     std::vector<Value> sol;        // show optimal solution
  //     wcsp_solver->getSolution(sol); // cost optimum

  //     //<! populates the decision rule
  //     for (iter = this->s->begin(); iter != this->s->end(); ++iter)
  //     {
  //       u1 = sol[vars[iter->first->at(0)->to_string()]];
  //       u2 = sol[vars[iter->first->at(1)->to_string()]];
  //       u = common::model->getJointActionIndex(std::vector<action>({u1, u2}));
  //       value += iter->second.first * this->getValueAt(iter->first, u);
  //       for(action v=0; v<common::model->getNumStates(); ++v)
  //       {
  //         a->setJointProbability(iter->first, v, 0.0);
  //       }
  //       a->setJointProbability(iter->first, u, 1.0);
  //     }
  //   }

  //   else
  //   {
  //     std::cout << "No solution found!" << std::endl;
  //     std::cout << wcsp_solver->getWCSP() << std::endl;
  //     exit(-1);
  //   }

  //   return a;
  }

  // template <typename TVector, typename TAction, typename TValue = double>
  // void q_vector<TVector,TAction,TValue>::setOffSet(number offset)
  // {
  //   this->offset = offset;
  // }

  template <typename TVector, typename TAction, typename TValue = double>
  int MaxPlanValueFunctionWCSP<TVector,TAction,TValue>::getCost(double value)
  {
    return (int)this->offset * (this->max - value);
  }

  // template <typename TVector, typename TAction, typename TValue = double>
  // void q_vector<TVector,TAction,TValue>::setJointTypes(const std::shared_ptr<State> &s)
  // {
  //   action u;
  //   this->max = std::numeric_limits<double>::lowest();

  //   base_q_vector<i_type, j_type, value_function>::setJointTypes(s);

  //   for (auto iter = this->s->begin(); iter != this->s->end(); ++iter)
  //   {
  //     for (u = 0; u < common::model->getNumActions(); u++)
  //     {
  //       this->max = std::max(max, iter->second.first * this->getValueAt(iter->first, u));
  //     }
  //   }
  // }
}