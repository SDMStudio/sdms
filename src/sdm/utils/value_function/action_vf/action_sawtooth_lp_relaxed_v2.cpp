// #include <sdm/utils/value_function/action_vf/action_sawtooth_lp_relaxed_v2.hpp>
// #include <sdm/utils/value_function/tabular_value_function.hpp>
// #include <sdm/core/state/interface/occupancy_state_interface.hpp>
// // #include <sdm/core/state/interface/serial_interface.hpp>

// // #include <sdm/world/base/mpomdp_interface.hpp>
// #include <sdm/world/occupancy_mdp.hpp>

// namespace sdm
// {
//     ActionVFSawtoothLPRelaxedV2::ActionVFSawtoothLPRelaxedV2() {}
//     ActionVFSawtoothLPRelaxedV2::ActionVFSawtoothLPRelaxedV2(const std::shared_ptr<SolvableByHSVI> &world, TypeOfResolution current_type_of_resolution,number bigM_value) : ActionVFSawtoothLP(world, current_type_of_resolution, bigM_value)
//     {
//     }

//     Pair<std::shared_ptr<Action>, double> ActionVFSawtoothLPRelaxedV2::selectBestAction(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t)
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

//                 //Resolve the WCSP problem
//                 auto [action,value] = this->createLP(vf,state, t);

//                 // We take the best action with the minimum value
//                 if (min_value >value)
//                 {
//                     min_value = value;
//                     best_action = action;
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
//             // exit(-1);
//         //     std::cout<<"Erreur "<<std::endl;
//         //     std::cout<<"LP value : "<<min_value<<", Bakcup Value : "<<vf->template backup<double>(state,best_action,t)<<std::endl;
//         }
//         // else
//         // {
//         //     std::cout<<"No problem"<<std::endl;
//         // }

//         return {best_action, min_value};
//     }

//     void ActionVFSawtoothLPRelaxedV2::createVariables(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t)
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

//             //<! Define variables \omega_k(x',o')
//             const auto &next_one_step_uncompressed_occupancy_state = this->support_point_set->toOccupancyState()->getOneStepUncompressedOccupancy();

//             // Go over all Joint History Next
//             for (const auto &next_joint_history : next_one_step_uncompressed_occupancy_state->getJointHistories())
//             {
//                 // Go over all Hidden State in the next one step uncomppresed occupancy state
//                 for (const auto next_hidden_state : next_one_step_uncompressed_occupancy_state->getBeliefAt(next_joint_history)->getStates())
//                 {
//                     // <! \omega_k(x',o')
//                     VarName = this->getVarNameWeightedStateJointHistory(next_one_step_uncompressed_occupancy_state, next_hidden_state, next_joint_history);
//                     var.add(IloBoolVar(env, 0, 1, VarName.c_str()));
//                     this->setNumber(VarName, index++);
//                 }
//             }

//             // Create Decentralized Variables
//             this->createDecentralizedVariables(vf, state, env, var, index, t);
//         }
//         catch (const std::exception &exc)
//         {
//             std::cerr << "SawtoothLPBackup::createVariables(..) exception caught: " << exc.what() << std::endl;
//             exit(-1);
//         }
//     }

//     void ActionVFSawtoothLPRelaxedV2::createConstraints(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloModel &model, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
//     {
//         assert(vf->getInitFunction() != nullptr);

//         //<!  Build sawtooth constraints v - \sum_{u} a(u|o) * Q(k,s,o,u,y,z, diff, t  ) + \omega_k(y,<o,z>)*M <= M,  \forall k, y,<o,z>
//         //<!  Build sawtooth constraints  Q(k,s,o,u,y,z, diff, t ) = (v_k - V_k) \frac{\sum_{x} s(x,o) * p(x,u,z,y)}}{s_k(y,<o,z>)},  \forall a(u|o)

//         try
//         {
//             auto compressed_occupancy_state = state->toOccupancyState();

//             if(vf->getSupport(t+1).empty())
//             {
//                 this->createInitialConstraints(vf, state,env, con, var,index, t);
//             }
//             else
//             {
//                 // Take the support at t+1
//                 auto ostate = this->support_point_set;

//                 const auto &next_one_step_uncompressed_occupancy_state = ostate->toOccupancyState()->getOneStepUncompressedOccupancy();

//                 // Compute the difference i.e. (v_k - V_k)
//                 double current_upper_bound = vf->getValueAt(ostate, t+1); //Get Value for the state at t+1
//                 double initial_upper_bound = vf->getInitFunction()->operator()(next_one_step_uncompressed_occupancy_state, t + 1); // Get initialisation of the state at t+1
                
//                 double difference = current_upper_bound - initial_upper_bound;
//                 // std::cout<<"Calcul differnece, current "<<current_upper_bound<<", initial "<<initial_upper_bound<<std::endl;

//                 // Go over all joint histories in over the support of next_one_step_uncompressed_occupancy_state
//                 for (const auto &next_joint_history : next_one_step_uncompressed_occupancy_state->getJointHistories())
//                 {
//                     // Go over all Hidden State in the Belief for a precise Joint History
//                     for (const auto &next_hidden_state : next_one_step_uncompressed_occupancy_state->getBeliefAt(next_joint_history)->getStates())
//                     {
//                         //Determine denominator of the sawtooth ratio
//                         double denominator = next_one_step_uncompressed_occupancy_state->getProbability(next_joint_history, next_hidden_state);

//                         // Determine the next Joint observation thanks to the next joint history
//                         auto next_joint_observation = this->determineNextJointObservation(compressed_occupancy_state, next_joint_history, t);

//                         // We search for the joint_history which allow us to obtain the current next_joint_history conditionning to the next joint observation
//                         switch (this->current_type_of_resolution_)
//                         {
//                         case TypeOfResolution::BigM:
//                             // this->createSawtoothBigM(compressed_occupancy_state,joint_history,next_hidden_state,next_joint_observation,next_joint_history,next_one_step_uncompressed_occupancy_state,probability,difference,env,con,var,index, t);
//                             break;
//                         case TypeOfResolution::IloIfThenResolution:
//                             this->createSawtoothIloIfThen(vf, compressed_occupancy_state,nullptr, next_hidden_state, next_joint_observation, next_joint_history, next_one_step_uncompressed_occupancy_state, denominator, difference, env, model, var, t);
//                             break;
//                         }
//                     }
//                 }
//                 this->createOmegaConstraints(ostate,env,con,var,index);
//             }
//             this->createDecentralizedConstraints(vf, state, env, con, var, index, t);
//         }
//         catch (const std::exception &exc)
//         {
//             std::cerr << "ActionVFSawtoothLP::createConstraints(..) exception caught: " << exc.what() << std::endl;
//             exit(-1);
//         }
//     }
// }
