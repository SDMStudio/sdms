// #include <sdm/utils/value_function/action_selection/action_sawtooth_lp_relaxed.hpp>
// #include <sdm/utils/value_function/point_set_qvalue_function.hpp>

// // #include <sdm/utils/value_function/vfunction/tabular_value_function.hpp>
// // #include <sdm/core/state/interface/occupancy_state_interface.hpp>
// // #include <sdm/core/state/private_occupancy_state.hpp>
// // #include <sdm/core/state/interface/serial_interface.hpp>

// // #include <sdm/world/base/mpomdp_interface.hpp>
// // #include <sdm/world/occupancy_mdp.hpp>

// namespace sdm
// {
//     ActionSelectionSawtoothLPRelaxed::ActionSelectionSawtoothLPRelaxed() {}
//     ActionSelectionSawtoothLPRelaxed::ActionSelectionSawtoothLPRelaxed(const std::shared_ptr<SolvableByHSVI>& world,TypeOfResolution current_type_of_resolution, number bigM_value, TypeSawtoothLinearProgram type_of_linear_program) : 
//         ActionSelectionSawtoothLP(world,current_type_of_resolution,bigM_value,type_of_linear_program)
//     {}

//     Pair<std::shared_ptr<Action>, double> ActionSelectionSawtoothLPRelaxed::selectBestActionRelaxedV2(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state,number t)
//     {
//         std::shared_ptr<Action> best_action;
//         double min_value = std::numeric_limits<double>::max();

//         if (vf->getSupport(t + 1).empty())
//         {
//             // Resolution of the problem when the support of Point Set is empty
//             this->all_support = std::unordered_map<Tuple<std::shared_ptr<State>,std::shared_ptr<Action>,double>,std::unordered_map<std::shared_ptr<HistoryInterface>,std::vector<std::shared_ptr<State>>>>();
//             auto best_action_AND_value = this->createLP(vf, state, t);

//             best_action = best_action_AND_value.first;
//             min_value = best_action_AND_value.second;
//         }
//         else
//         {

//             // For the Full version of Sawtooth, wo over all the Point Set  
//             for (const auto &ostate_AND_action_AND_value : std::dynamic_pointer_cast<PointSetQValueFunction>(vf)->getRepresentation(t+1))
//             {
//                 const auto &next_one_step_uncompressed_occupancy_state = ostate_AND_action_AND_value.first->toOccupancyState()->getOneStepUncompressedOccupancy();

//                 this->all_support = std::unordered_map<Tuple<std::shared_ptr<State>,std::shared_ptr<Action>,double>,std::unordered_map<std::shared_ptr<HistoryInterface>,std::vector<std::shared_ptr<State>>>>();
//                 this->all_support.emplace(ostate_AND_action_AND_value, std::unordered_map<std::shared_ptr<HistoryInterface>,std::vector<std::shared_ptr<State>>>());

//                 // Go over all Joint History Next
//                 for (const auto &next_joint_history : next_one_step_uncompressed_occupancy_state->getJointHistories())
//                 {
//                     this->all_support[ostate_AND_action_AND_value].emplace(next_joint_history,std::vector<std::shared_ptr<State>>());
//                     // Go over all Hidden State in the next one step uncomppresed occupancy state
//                     for (const auto next_hidden_state : next_one_step_uncompressed_occupancy_state->getBeliefAt(next_joint_history)->getStates())
//                     {
//                         this->all_support[ostate_AND_action_AND_value][next_joint_history].push_back(next_hidden_state);
//                     }
//                 }

//                 auto [action,value] = this->createLP(vf,state, t);

//                 if (min_value >value)
//                 {
//                     min_value = value;
//                     best_action = action;
//                 }
//             }
//         }
//         return {best_action, min_value};
//     }

//     Pair<std::shared_ptr<Action>, double> ActionSelectionSawtoothLPRelaxed::selectBestActionRelaxed(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state,number t)
//     {
//         std::shared_ptr<Action> best_action;
//         double min_value = std::numeric_limits<double>::max();

//         if (vf->getSupport(t + 1).empty())
//         {
//             // Resolution of the problem when the support of Point Set is empty
//             this->all_support = std::unordered_map<Tuple<std::shared_ptr<State>,std::shared_ptr<Action>,double>,std::unordered_map<std::shared_ptr<HistoryInterface>,std::vector<std::shared_ptr<State>>>>();
//             auto best_action_AND_value = this->createLP(vf, state, t);

//             best_action = best_action_AND_value.first;
//             min_value = best_action_AND_value.second;
//         }
//         else
//         {

//             // For the Full version of Sawtooth, wo over all the Point Set  
//             std::cout<<"New Selection of action set"<<std::endl;

//             for (const auto &ostate_AND_action_AND_value : std::dynamic_pointer_cast<TabularValueFunction>(vf)->getRepresentation(t+1))
//             {
//                 const auto &next_one_step_uncompressed_occupancy_state = ostate_AND_action_AND_value.first->toOccupancyState()->getOneStepUncompressedOccupancy();

//                 double max_value_support = -std::numeric_limits<double>::max();
//                 std::shared_ptr<Action> best_action_support;

//                 std::cout<<"New point set"<<std::endl;

//                 // Go over all Joint History Next
//                 for (const auto &next_joint_history : next_one_step_uncompressed_occupancy_state->getJointHistories())
//                 {
//                     // Go over all Hidden State in the next one step uncomppresed occupancy state
//                     for (const auto next_hidden_state : next_one_step_uncompressed_occupancy_state->getBeliefAt(next_joint_history)->getStates())
//                     {
//                         this->all_support = std::unordered_map<Tuple<std::shared_ptr<State>,std::shared_ptr<Action>,double>,std::unordered_map<std::shared_ptr<HistoryInterface>,std::vector<std::shared_ptr<State>>>>();
//                         this->all_support.emplace(ostate_AND_action_AND_value, std::unordered_map<std::shared_ptr<HistoryInterface>,std::vector<std::shared_ptr<State>>>());
//                         this->all_support[ostate_AND_action_AND_value].emplace(next_joint_history,std::vector<std::shared_ptr<State>>());
//                         this->all_support[ostate_AND_action_AND_value][next_joint_history].push_back(next_hidden_state);

//                         auto [action,value] = this->createLP(vf,state, t);

//                         std::cout<<"Value Found "<<value<<std::endl;

//                         // We take the best action with the minimum value
//                         if (max_value_support < value)
//                         {
//                             max_value_support = value;
//                             best_action_support = action->toAction();
//                         }
//                     }
//                 }

//                 std::cout<<"Best value for this point set "<<max_value_support<<std::endl;

//                 if (min_value >max_value_support)
//                 {
//                     min_value = max_value_support;
//                     best_action = best_action_support;
//                 }
//             }
//         }
//         auto occupancy_mdp = std::static_pointer_cast<OccupancyMDP>(ActionSelectionBase::world_);
//         auto under_pb = std::dynamic_pointer_cast<MPOMDPInterface>(ActionSelectionBase::world_->getUnderlyingProblem());


//         std::cout<<"Action "<<best_action->str()<<std::endl;
//         // std::cout<<"Next occupancy "<<occupancy_mdp->nextOccupancyState(state, best_action, nullptr, t)->str()<<std::endl;

//         std::cout<<"Value LP "<<min_value<<std::endl;
//         std::cout<<"Q Value "<<occupancy_mdp->getReward(state,best_action,t) +vf->getValueAt(occupancy_mdp->nextOccupancyState(state, best_action, nullptr, t),t+1);
//         std::cout<<"Value Evaluation "<<vf->evaluate(state,t).second<<std::endl; 


//         // if(std::abs(min_value - resultat2)>0.01)
//         // {
//         //     std::cout<<"NExt occupancy "<<occupancy_mdp->nextOccupancyState(state, best_action, nullptr, t)->str()<<std::endl;
//         //     std::cout<<"Value LP "<<min_value<<std::endl;
//         //     std::cout<<"Value Backup "<<resultat2<<std::endl;
//         //     exit(-1);
//         // }

//         return {best_action, min_value};
//     }


//     Pair<std::shared_ptr<Action>, double> ActionSelectionSawtoothLPRelaxed::selectBestAction(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t)
//     {
//         switch (this->type_of_linear_program_)
//         {
//         case TypeSawtoothLinearProgram::PLAIN_SAWTOOTH_LINER_PROGRAMMING :
//             return this->selectBestActionFull(vf,state,t);
//             break;
//         case TypeSawtoothLinearProgram::RELAXED_SAWTOOTH_LINER_PROGRAMMING :
//             return this->selectBestActionRelaxed(vf,state,t);
//             break;
//         case TypeSawtoothLinearProgram::RELAXED_V2_SAWTOOTH_LINER_PROGRAMMING :
//             return this->selectBestActionRelaxedV2(vf,state,t);
//             break;

//         default:
//             break;
//         }
//     }

//     // ************************************************************************
//     // ************************************************************************
//     // ******************* Create Variable for LP *****************************
//     // ************************************************************************
//     // ************************************************************************

//     void ActionSelectionSawtoothLPRelaxed::createOmegaVariable(IloEnv &env, IloNumVarArray &var, number &index)
//     {
//         //<! Define variables \omega_k(x',o')

//         std::string VarName;

//         // Go over all Point Set in t+1
//         for (const auto &next_state_AND_value_AND_All_next_history_AND_All_next_hidden_state : this->all_support)
//         {
//             const auto &ostate = next_state_AND_value_AND_All_next_history_AND_All_next_hidden_state.first.first->toOccupancyState();

//             // Go over all Joint History Next
//             for (const auto &next_history_AND_All_next_hidden_state : next_state_AND_value_AND_All_next_history_AND_All_next_hidden_state.second)
//             {
//                 const auto &next_history = next_history_AND_All_next_hidden_state.first->toJointHistory();

//                 // Go over all Hidden State in the next one step uncomppresed occupancy state
//                 for (const auto next_hidden_state : next_history_AND_All_next_hidden_state.second)
//                 {
//                     // <! \omega_k(x',o')
//                     VarName = this->getVarNameWeightedStateJointHistory(ostate, next_hidden_state, next_history);
//                     var.add(IloBoolVar(env, 0, 1, VarName.c_str()));
//                     this->setNumber(VarName, index++);
//                 }
//             }
//         }
//     }

//     // ************************************************************************
//     // ************************************************************************
//     // ******************* Create Constraint for LP ***************************
//     // ************************************************************************
//     // ************************************************************************

//     void ActionSelectionSawtoothLPRelaxed::createConstraints(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloModel &model, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
//     {
//         assert(vf->getInitFunction() != nullptr);

//         //<!  Build sawtooth constraints v - \sum_{u} a(u|o) * Q(k,s,o,u,y,z, diff, t  ) + \omega_k(y,<o,z>)*M <= M,  \forall k, y,<o,z>
//         //<!  Build sawtooth constraints  Q(k,s,o,u,y,z, diff, t ) = (v_k - V_k) \frac{\sum_{x} s(x,o) * p(x,u,z,y)}}{s_k(y,<o,z>)},  \forall a(u|o)

//         try
//         {
//             auto compressed_occupancy_state = state->toOccupancyState();

//             // this->createGlobalConstraint(vf, state, env, con, var, index, t);

//             if(vf->getSupport(t+1).empty())
//             {
//                 this->createInitialConstraints(vf, state,env, con, var,index, t);
//                 this->createInitialConstraints2(vf, state,env, con, var,index, t);
//             }
//             else
//             {
//                 // Go over all points in the point set at t+1
//                 for (const auto &next_state_AND_value_AND_All_next_history_AND_All_next_hidden_state : this->all_support)
//                 {
//                     const auto &ostate = next_state_AND_value_AND_All_next_history_AND_All_next_hidden_state.first.first->toOccupancyState();
//                     const auto &next_one_step_uncompressed_occupancy_state = ostate->toOccupancyState()->getOneStepUncompressedOccupancy();

//                     // Compute the difference i.e. (v_k - V_k)
//                     double difference = this->computeDifference(vf,next_state_AND_value_AND_All_next_history_AND_All_next_hidden_state.first,t);

//                     // Go over all joint histories in over the support of next_one_step_uncompressed_occupancy_state
//                     for (const auto &next_history_AND_All_next_hidden_state : next_state_AND_value_AND_All_next_history_AND_All_next_hidden_state.second)
//                     {
//                         const auto &next_history = next_history_AND_All_next_hidden_state.first->toJointHistory();

//                         // Go over all Hidden State in the Belief for a precise Joint History
//                         for (const auto &next_hidden_state : next_history_AND_All_next_hidden_state.second)
//                         {
//                             //Determine denominator of the sawtooth ratio
//                             double denominator = next_one_step_uncompressed_occupancy_state->getProbability(next_history, next_hidden_state);

//                             // Determine the next Joint observation thanks to the next joint history
//                             auto next_joint_observation = this->determineNextJointObservation(next_history, t);

//                             this->createConstraintsKnowingInformation(vf, state, nullptr, next_hidden_state, next_joint_observation, next_history, ostate, denominator, difference, env,model, con, var, index, t);
//                         }
//                     }
//                     this->createOmegaConstraints(next_state_AND_value_AND_All_next_history_AND_All_next_hidden_state.first,env,con,var,index);
//                 }
//             }
//             this->createDecentralizedConstraints(vf, state, env, con, var, index, t);
//         }
//         catch (const std::exception &exc)
//         {
//             std::cerr << "ActionSelectionSawtoothLPRelaxed::createConstraints(..) exception caught: " << exc.what() << std::endl;
//             exit(-1);
//         }
//     }

//     void ActionSelectionSawtoothLPRelaxed::createConstraintsKnowingInformation(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> &, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_joint_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double denominator , double difference , IloEnv &env, IloModel &model, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
//     {
//         // We search for the joint_history which allow us to obtain the current next_history conditionning to the next joint observation
//         switch (this->current_type_of_resolution_)
//         {
//         case TypeOfResolution::BigM:
//             this->createSawtoothBigM(vf,state,nullptr,next_hidden_state,next_joint_observation,next_joint_history, next_state,denominator,difference,env,con,var,index, t);
//             // this->createSawtoothBigM2(vf,state,nullptr,next_hidden_state,next_joint_observation,next_joint_history, next_state,denominator,difference,env,con,var,index, t);
//             break;
//         case TypeOfResolution::IloIfThenResolution:
//             this->createSawtoothIloIfThen(vf, state,nullptr, next_hidden_state, next_joint_observation, next_joint_history,  next_state, denominator, difference, env, model, var, t);
//             // this->createSawtoothIloIfThen2(vf, state,nullptr, next_hidden_state, next_joint_observation, next_joint_history,  next_state, denominator, difference, env, model, var, t);
//             break;
//         }
//     }

//     void ActionSelectionSawtoothLPRelaxed::createOmegaConstraints(const Tuple<std::shared_ptr<State>,std::shared_ptr<Action>,double> &state_AND_value, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index)
//     {
//         const auto &ostate = state_AND_value.first->toOccupancyState();

//         number recover = 0;

//         // Build constraint \sum{x',o'} \omega_k(x',o') = 1
//         con.add(IloRange(env, 1.0, 1.0));

//         // Go over all joint histories in over the support of next_one_step_uncompressed_occupancy_state
//         for (const auto &next_history_AND_All_next_hidden_state :  this->all_support[state_AND_value])
//         {
//             const auto &next_history = next_history_AND_All_next_hidden_state.first->toJointHistory();

//             for (const auto &next_hidden_state : next_history_AND_All_next_hidden_state.second)
//             {
//                 // <! \omega_k(x',o')
//                 auto VarName = this->getVarNameWeightedStateJointHistory(ostate, next_hidden_state, next_history);
//                 recover = this->getNumber(VarName);
//                 con[index].setLinearCoef(var[recover], +1.0);
//             }
//         }
//         index++;
//     }
    
//     void ActionSelectionSawtoothLPRelaxed::createInitialConstraints(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state,  IloEnv &env,IloRangeArray &con, IloNumVarArray &var,number &index, number t)
//     {
//         auto under_pb = ActionSelectionBase::world_->getUnderlyingProblem();

//         number recover = 0;
//         double Qrelaxation;

//         con.add(IloRange(env, -IloInfinity, 0));
//         con[index].setLinearCoef(var[this->getNumber(this->getVarNameWeight(0))], +1.0);

//         // Go over all actions
//         for (const auto &action : *under_pb->getActionSpace(t))
//         {
//             for(const auto& joint_history : state->toOccupancyState()->getJointHistories())
//             {
//                 //<! 1.c.4 get variable a(u|o) and set constant
//                 recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action->toAction(), joint_history));

//                 Qrelaxation = this->getQValueRelaxation(vf, state, joint_history, action->toAction(), t);
//                 con[index].setLinearCoef(var[recover], -Qrelaxation);
//             }
//         }
//         index++;
//     }

//     void ActionSelectionSawtoothLPRelaxed::createInitialConstraints2(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state,  IloEnv &env,IloRangeArray &con, IloNumVarArray &var,number &index, number t)
//     {
//         auto under_pb = ActionSelectionBase::world_->getUnderlyingProblem();

//         number recover = 0;
//         double Qrelaxation;

//         con.add(IloRange(env, -IloInfinity, vf->getValueAt(state) - 0.001));

//         // Go over all actions
//         for (const auto &action : *under_pb->getActionSpace(t))
//         {
//             for(const auto& joint_history : state->toOccupancyState()->getJointHistories())
//             {
//                 //<! 1.c.4 get variable a(u|o) and set constant
//                 recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action->toAction(), joint_history));

//                 Qrelaxation = this->getQValueRelaxation(vf, state, joint_history, action->toAction(), t);
//                 con[index].setLinearCoef(var[recover], Qrelaxation);
//             }
//         }
//         index++;
//     }

//     void ActionSelectionSawtoothLPRelaxed::createSawtoothBigM(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> &, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double denominator , double difference , IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
//     {
//         try
//         {
//             auto under_pb = ActionSelectionBase::world_->getUnderlyingProblem();

//             number recover = 0;

//             con.add(IloRange(env, -IloInfinity, this->bigM_value_));
//             con[index].setLinearCoef(var[this->getNumber(this->getVarNameWeight(0))], +1.0);

//             // Go over all actions
//             for (const auto &action : *under_pb->getActionSpace(t))
//             {
//                 for(const auto &joint_history : state->toOccupancyState()->getJointHistories())
//                 {
//                     //<! 1.c.4 get variable a(u|o) and set constant
//                     recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action->toAction(), joint_history));

//                     //<! 1.c.4 get variable a(u|o) and set constant
//                     con[index].setLinearCoef(var[recover],-this->computeSawtooth(vf,state, action->toAction(),joint_history,next_hidden_state,next_observation,next_joint_history,denominator,difference,t) );
//                 }
//             }
//             // <! \omega_k(x',o') * BigM
//             recover = this->getNumber(this->getVarNameWeightedStateJointHistory(next_state, next_hidden_state, next_joint_history));
//             con[index].setLinearCoef(var[recover], this->bigM_value_);

//             index++;
//         }
//         catch (const std::exception &exc)
//         {
//             // catch anything thrown within try block that derives from std::exception
//             std::cerr << "SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedySawtoothBigM(..) exception caught: " << exc.what() << std::endl;
//             exit(-1);
//         }
//     }



//     void ActionSelectionSawtoothLPRelaxed::createSawtoothIloIfThen(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> &, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double denominator, double difference, IloEnv &env, IloModel &model, IloNumVarArray &var, number t)
//     {
//         try
//         {
//             auto under_pb = ActionSelectionBase::world_->getUnderlyingProblem();
//             number recover = 0;

//             IloExpr expr(env);
//             //<! 1.c.1 get variable v and set coefficient of variable v
//             expr = var[this->getNumber(this->getVarNameWeight(0))];

//             // Go over all actions
//             for (const auto &action : *under_pb->getActionSpace(t))
//             {
//                 for(const auto &joint_history : state->toOccupancyState()->getJointHistories())
//                 {
//                     recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action->toAction(), joint_history));

//                     //<! 1.c.4 get variable a(u|o) and set constant
//                     expr -=  var[recover] * this->computeSawtooth(vf,state,action->toAction(),joint_history,next_hidden_state,next_observation,next_joint_history,denominator,difference,t);
//                 }
//             }

//             // <! get variable \omega_k(x',o')
//             recover = this->getNumber(this->getVarNameWeightedStateJointHistory(next_state, next_hidden_state, next_joint_history));
//             model.add(IloIfThen(env, var[recover] > 0, expr <= 0));
//         }
//         catch (const std::exception &exc)
//         {
//             // catch anything thrown within try block that derives from std::exception
//             std::cerr << "SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedySawtoothIloIfThen(..) exception caught: " << exc.what() << std::endl;
//             exit(-1);
//         }
//     }


//     void ActionSelectionSawtoothLPRelaxed::createSawtoothBigM2(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> &, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double denominator , double difference , IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
//     {
//         try
//         {
//             // v_ + espilon + K >= f() + K * omega_(x',o')


//             auto under_pb = ActionSelectionBase::world_->getUnderlyingProblem();

//             number recover = 0;

//             con.add(IloRange(env, -IloInfinity, vf->getValueAt(state) + 0.01 + this->bigM_value_));

//             // Go over all actions
//             for (const auto &action : *under_pb->getActionSpace(t))
//             {
//                 for(const auto &joint_history : state->toOccupancyState()->getJointHistories())
//                 {
//                     //<! 1.c.4 get variable a(u|o) and set constant
//                     recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action->toAction(), joint_history));

//                     //<! 1.c.4 get variable a(u|o) and set constant
//                     con[index].setLinearCoef(var[recover],this->computeSawtooth(vf,state, action->toAction(),joint_history,next_hidden_state,next_observation,next_joint_history,denominator,difference,t) );
//                 }
//             }
//             // <! \omega_k(x',o') * BigM
//             recover = this->getNumber(this->getVarNameWeightedStateJointHistory(next_state, next_hidden_state, next_joint_history));
//             con[index].setLinearCoef(var[recover], this->bigM_value_);

//             index++;
//         }
//         catch (const std::exception &exc)
//         {
//             // catch anything thrown within try block that derives from std::exception
//             std::cerr << "SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedySawtoothBigM(..) exception caught: " << exc.what() << std::endl;
//             exit(-1);
//         }
//     }

//     void ActionSelectionSawtoothLPRelaxed::createSawtoothIloIfThen2(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> &, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double denominator, double difference, IloEnv &env, IloModel &model, IloNumVarArray &var, number t)
//     {
//         try
//         {
//             auto under_pb = ActionSelectionBase::world_->getUnderlyingProblem();
//             number recover = 0;

//             IloExpr expr(env);

//             // Go over all actions
//             for (const auto &action : *under_pb->getActionSpace(t))
//             {
//                 for(const auto &joint_history : state->toOccupancyState()->getJointHistories())
//                 {
//                     recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action->toAction(), joint_history));

//                     //<! 1.c.4 get variable a(u|o) and set constant
//                     expr +=  var[recover] * this->computeSawtooth(vf,state,action->toAction(),joint_history,next_hidden_state,next_observation,next_joint_history,denominator,difference,t);
//                 }
//             }

//             // <! get variable \omega_k(x',o')
//             recover = this->getNumber(this->getVarNameWeightedStateJointHistory(next_state, next_hidden_state, next_joint_history));
//             model.add(IloIfThen(env, var[recover] > 0, expr <= vf->getValueAt(state) + 0.0001));
//         }
//         catch (const std::exception &exc)
//         {
//             // catch anything thrown within try block that derives from std::exception
//             std::cerr << "SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedySawtoothIloIfThen(..) exception caught: " << exc.what() << std::endl;
//             exit(-1);
//         }
//     }
// }
