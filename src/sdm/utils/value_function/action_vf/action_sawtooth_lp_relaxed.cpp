// #include <sdm/utils/value_function/action_vf/action_sawtooth_lp_relaxed.hpp>
// #include <sdm/utils/value_function/tabular_value_function.hpp>
// #include <sdm/core/state/interface/occupancy_state_interface.hpp>
// // #include <sdm/core/state/interface/serial_interface.hpp>

// // #include <sdm/world/base/mpomdp_interface.hpp>
// #include <sdm/world/occupancy_mdp.hpp>

// namespace sdm
// {
//     ActionVFSawtoothLPRelaxed::ActionVFSawtoothLPRelaxed() {}
//     ActionVFSawtoothLPRelaxed::ActionVFSawtoothLPRelaxed(const std::shared_ptr<SolvableByHSVI> &world, TypeOfResolution current_type_of_resolution) : ActionVFSawtoothLP(world, current_type_of_resolution, 0)
//     {
//         this->current_type_of_resolution_ = TypeOfResolution::BigM;
//     }

//     Pair<std::shared_ptr<Action>, double> ActionVFSawtoothLPRelaxed::selectBestAction(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t)
//     {
//         std::shared_ptr<Action> best_action;
//         double min_value = std::numeric_limits<double>::max();

//         if (vf->getSupport(t + 1).empty())
//         {
//             // Resolution of the problem when the support of Point Set is empty
//             // std::cout<<"No support"<<std::endl;
//             this->support_empty = true;
//             auto best_action_AND_value = this->createLP(vf, state, t);

//             best_action = best_action_AND_value.first;
//             min_value = best_action_AND_value.second;
//         }
//         else
//         {
//             this->support_empty = false;

//             std::cout<<"Size "<<vf->getSupport(t+1).size()<<std::endl;
//             for (const auto &point : vf->getSupport(t+1))
//             {
//                 std::cout<<"New point "<<point->str()<<std::endl;
//                 this->support_point_set = point;

//                 // We go over the support of the point i.e. we go over the history and the action
//                 // auto decision_rule_associed = this->state_linked_to_decision_rule.at(point_AND_value.first)->toDecisionRule();

//                 double max_value_support = -std::numeric_limits<double>::max();
//                 std::shared_ptr<Action> best_action_support;

//                 auto next_uncompressed_occupancy_state = point->toOccupancyState()->getOneStepUncompressedOccupancy();

//                 // Go over all joint histories in over the support of next_one_step_uncompressed_occupancy_state
//                 for (const auto& next_joint_history_of_support : next_uncompressed_occupancy_state->getJointHistories())
//                 {
//                     for(const auto& next_hidden_state : next_uncompressed_occupancy_state->getBeliefAt(next_joint_history_of_support)->getStates())
//                     {
//                         // Associate the variable next joint history and the action to the support 
//                         this->support_of_the_next_history = next_joint_history_of_support;
//                         this->support_of_the_next_hidden_state = next_hidden_state;

//                         //Resolve the WCSP problem
//                         auto [action,value] = this->createLP(vf,state, t);

//                         std::cout<<"Value found "<<value<<std::endl;
//                         // if(value>-4 and vf->getSupport(t+1).size()>1)
//                         // {
//                         //     system("cat lb_bellman_op.lp");
//                         // }

//                         // We take the best action with the minimum value
//                         if (max_value_support < value)
//                         {
//                             max_value_support = value;
//                             best_action_support = action->toAction();
//                         }
//                     }
//                 }
//                 // We take the best action with the minimum value
//                 if (min_value >max_value_support)
//                 {
//                     min_value = max_value_support;
//                     best_action = best_action_support;
//                 }
//             }
//         }
//         // Save the best action associed to a state

//         auto occupancy_mdp = std::static_pointer_cast<OccupancyMDP>(ActionVFBase::world_);

//         double reward =0.0;
//         for (const auto &joint_history : state->toOccupancyState()->getOneStepUncompressedOccupancy()->getJointHistories())
//         {
//             // Get the belief corresponding to this history
//             auto belief = state->toOccupancyState()->getOneStepUncompressedOccupancy()->getBeliefAt(joint_history);
//             // Get the action from decision rule
//             auto joint_action = occupancy_mdp->applyDecisionRule(state->toOccupancyState()->getOneStepUncompressedOccupancy(), state->toOccupancyState()->getCompressedJointHistory(joint_history), best_action, t);
//             // Update the expected reward
//             reward += state->toOccupancyState()->getOneStepUncompressedOccupancy()->getProbability(joint_history) * occupancy_mdp->getUnderlyingBeliefMDP()->getReward(belief, joint_action, t);
//         }


//         double resultat2 = reward +  vf->evaluate(occupancy_mdp->nextOccupancyState(state, best_action, nullptr, t)->toOccupancyState()->getOneStepUncompressedOccupancy(),t+1).second;




//         std::cout<<"Action "<<best_action->str()<<std::endl;
//         std::cout<<"Value LP "<<min_value<<std::endl;
//         std::cout<<"Value Backup "<<resultat2<<std::endl;
//         if(std::abs(min_value - resultat2)>0.01)
//         {
//             exit(-1);
//         //     std::cout<<"Erreur "<<std::endl;
//         //     std::cout<<"LP value : "<<min_value<<", Bakcup Value : "<<vf->template backup<double>(state,best_action,t)<<std::endl;
//         }
//         // else
//         // {
//         //     std::cout<<"No problem"<<std::endl;
//         // }

//         return {best_action, min_value};
//     }

//     void ActionVFSawtoothLPRelaxed::createVariables(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t)
//     {
//         try
//         {
//             //<! tracking variables
//             std::string VarName;

//             this->variables.clear();

//             //<! 0.b Build variables v_0 = objective variable!
//             VarName = this->getVarNameWeight(0);
//             var.add(IloNumVar(env, -IloInfinity, vf->getValueAt(state,t), VarName.c_str()));
//             this->setNumber(VarName, index++);

//             // Create Decentralized Variables
//             this->createDecentralizedVariables(vf, state, env, var, index, t);
//         }
//         catch (const std::exception &exc)
//         {
//             std::cerr << "SawtoothLPBackup::createVariables(..) exception caught: " << exc.what() << std::endl;
//             exit(-1);
//         }
//     }

//     void ActionVFSawtoothLPRelaxed::createConstraints(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloModel &, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
//     {
//         assert(vf->getInitFunction() != nullptr);

//         //<!  Build sawtooth constraints v - \sum_{u} a(u|o) * Q(k,s,o,u,y,z, diff, t  ) + \omega_k(y,<o,z>)*M <= M,  \forall k, y,<o,z>
//         //<!  Build sawtooth constraints  Q(k,s,o,u,y,z, diff, t ) = (v_k - V_k) \frac{\sum_{x} s(x,o) * p(x,u,z,y)}}{s_k(y,<o,z>)},  \forall a(u|o)

//         try
//         {
//             auto compressed_occupancy_state = state->toOccupancyState();

//             if(this->support_empty)
//             {
//                 this->createInitialConstraints(vf, state,env, con, var,index, t);
//             }
//             else
//             {
//                 auto ostate = this->support_point_set;
//                 auto next_joint_history = this->support_of_the_next_history->toJointHistory();
//                 auto next_hidden_state = this->support_of_the_next_hidden_state;

//                 const auto &next_one_step_uncompressed_occupancy_state = ostate->toOccupancyState()->getOneStepUncompressedOccupancy();

//                 // Compute the difference i.e. (v_k - V_k)
//                 double current_upper_bound = vf->getValueAt(ostate, t+1); //Get Value for the state at t+1
//                 double initial_upper_bound = vf->getInitFunction()->operator()(next_one_step_uncompressed_occupancy_state, t + 1); // Get initialisation of the state at t+1
                
//                 double difference = current_upper_bound - initial_upper_bound;

        
//                 //Determine denominator of the sawtooth ratio
//                 double denominator = next_one_step_uncompressed_occupancy_state->getProbability(next_joint_history, next_hidden_state);

//                 // Determine the next Joint observation thanks to the next joint history
//                 auto next_joint_observation = this->determineNextJointObservation(compressed_occupancy_state, next_joint_history, t);

//                 this->createSawtoothBigM(vf,compressed_occupancy_state,nullptr,next_hidden_state,next_joint_observation,next_joint_history,next_one_step_uncompressed_occupancy_state,denominator,difference,env,con,var,index, t);
//             }
//             this->createDecentralizedConstraints(vf, state, env, con, var, index, t);
//         }
//         catch (const std::exception &exc)
//         {
//             std::cerr << "ActionVFSawtoothLP::createConstraints(..) exception caught: " << exc.what() << std::endl;
//             exit(-1);
//         }
//     }

//     // *********************************************
//     // Specialisation for the Occupancy State
//     // *********************************************

//     void ActionVFSawtoothLPRelaxed::createSawtoothBigM(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> &, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &, double denominator, double difference, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
//     {
//         try
//         {
//             auto under_pb = ActionVFBase::world_->getUnderlyingProblem();

//             number recover = 0;
//             double Qrelaxation,SawtoothRatio;

//             con.add(IloRange(env, -IloInfinity, this->bigM_value_));
//             con[index].setLinearCoef(var[this->getNumber(this->getVarNameWeight(0))], +1.0);

//             // Go over all actions
//             for (const auto &action : *under_pb->getActionSpace(t))
//             {
//                 for(const auto &joint_history : state->toOccupancyState()->getJointHistories())
//                 {
//                     //<! 1.c.4 get variable a(u|o) and set constant
//                     recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action->toAction(), joint_history));
//                     // con[index].setLinearCoef(var[recover], -this->getQValueRealistic(vf, state, joint_history, action->toAction(), next_hidden_state, next_observation, probability, difference, t));
//                     Qrelaxation = this->getQValueRelaxation(vf, state, joint_history, action->toAction(), t);

//                     if(joint_history->expand(next_observation) == next_joint_history)
//                     {
//                         SawtoothRatio = this->getSawtoothMinimumRatio(vf, state, joint_history, action->toAction(), next_hidden_state, next_observation, denominator, t);
//                     }else
//                     {
//                         SawtoothRatio = 0;
//                     }
//                     // std::cout<<"Value Relaxation "<<Qrelaxation<<std::endl;
//                     // std::cout<<"Value SawtoothRatio "<<SawtoothRatio<<std::endl;

//                     //<! 1.c.4 get variable a(u|o) and set constant
//                     con[index].setLinearCoef(var[recover],-(Qrelaxation + SawtoothRatio * difference));
//                 }
//             }
//             index++;
//         }
//         catch (const std::exception &exc)
//         {
//             // catch anything thrown within try block that derives from std::exception
//             std::cerr << "SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedySawtoothBigM(..) exception caught: " << exc.what() << std::endl;
//             exit(-1);
//         }
//     }
// }
