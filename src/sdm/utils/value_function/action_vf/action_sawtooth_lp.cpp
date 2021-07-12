// #include <sdm/utils/value_function/action_vf/action_sawtooth_lp.hpp>
// #include <sdm/utils/value_function/tabular_value_function.hpp>
// #include <sdm/core/state/interface/occupancy_state_interface.hpp>
// #include <sdm/core/state/interface/serial_interface.hpp>
// #include <sdm/world/base/mpomdp_interface.hpp>

// namespace sdm
// {
//     ActionVFSawtoothLP::ActionVFSawtoothLP() {}
//     ActionVFSawtoothLP::ActionVFSawtoothLP(const std::shared_ptr<SolvableByHSVI>& world,TypeOfResolution current_type_of_resolution, number bigM_value, TypeSawtoothLinearProgram type_sawtooth_resolution) : 
//         ActionVFBase<double>(world),DecentralizedLP(world) , csawtooth_lp_(type_sawtooth_resolution), current_type_of_resolution_(current_type_of_resolution)
//     {
//         switch (type_sawtooth_resolution)
//         {
//         case TypeSawtoothLinearProgram::RELAXED_SAWTOOTH_LINER_PROGRAMMING :
//             this->bigM_value_ = 0;
//             break;
        
//         default:
//             this->bigM_value_ = bigM_value;
//             break;
//         }
//     }

//     TypeSawtoothLinearProgram ActionVFSawtoothLP::getSawtoothType()
//     {
//         return this->csawtooth_lp_;
//     }

//     void ActionVFSawtoothLP::setSawtoothType(const TypeSawtoothLinearProgram &csawtooth_lp)
//     {
//         this->csawtooth_lp_ = csawtooth_lp;
//     }


//     Pair<std::shared_ptr<Action>,double> ActionVFSawtoothLP::selectBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t)
//     {
//         switch (this->getSawtoothType())
//         {
//         case TypeSawtoothLinearProgram::PLAIN_SAWTOOTH_LINER_PROGRAMMING:
//             return this->createFullSawtooth(vf,state, t);
//         case TypeSawtoothLinearProgram::RELAXED_SAWTOOTH_LINER_PROGRAMMING:
//             return this->createRelaxedSawtooth(vf,state, t);
//         default:
//             return this->createFullSawtooth(vf,state, t);
//         }
//     }

//     Pair<std::shared_ptr<Action>,double> ActionVFSawtoothLP::createRelaxedSawtooth(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t)
//     {
//         std::shared_ptr<Action> best_action, action;
//         double value;

//         value = std::numeric_limits<double>::max();

//         if (vf->getSupport(t + 1).empty())
//         {
//             this->tmp_representation = {};
//             // best_action = this->getGreedy(state, t).first;
//         }

//         else
//         {
//             // for (const auto &element : vf->getSupport(t + 1))
//             // {
//             //     // this->tmp_representation = {element};

//             //     // action = this->getGreedy(state, t);

//             //     // value = this->getQValueAt(state, action, t);

//             //     //if( this->representation[t+1].size() > 1 ) std::cout << "\thorizon=" << t << "\tvalue=" << value << "\t cub=" << cub << "\t vub=" << vub << std::endl;

//             //     if (cub > value)
//             //     {
//             //         cub = value;
//             //         best_action = action;
//             //     }
//             // }
//         }
//         return std::make_pair(best_action,value);
//     }

//     Pair<std::shared_ptr<Action>,double> ActionVFSawtoothLP::createFullSawtooth(const std::shared_ptr<ValueFunction>&vf,const std::shared_ptr<State> &state, number t)
//     {
//         // this->tmp_representation = std::make_shared<MappedVector<std::shared_ptr<State>,double>>>(std::static_pointer_cast<TabularValueFunction>(vf)->getRepresentation(t + 1));
//         return this->createLP(vf,state, t);
//     }

//     void ActionVFSawtoothLP::createVariables(const std::shared_ptr<ValueFunction>&vf,const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number t)
//     {
//         try
//         {
//             //<! tracking variable ids
//             number index = 0;

//             //<! tracking variables
//             std::string VarName;

//             this->variables.clear();

//             //<! 0.b Build variables v_0 = objective variable!
//             VarName = this->getVarNameWeight(0);
//             var.add(IloNumVar(env, -IloInfinity, 0.0, VarName.c_str()));
//             this->setNumber(VarName, index++);

//             //<! Define variables \omega_k(x',o')

//             // Go over all Point Set in t+1
//             for (const auto &element_state_AND_upper_bound : this->tmp_representation)
//             {
//                 const auto &next_one_step_uncompressed_occupancy_state = element_state_AND_upper_bound.first->toOccupancyState();

//                 // Go over all Joint History Next
//                 for (const auto &joint_history : next_one_step_uncompressed_occupancy_state->getJointHistories())
//                 {
//                     for(const auto &hidden_state : next_one_step_uncompressed_occupancy_state->getStatesAt(joint_history))
//                     {
//                         // <! \omega_k(x',o')
//                         VarName = this->getVarNameWeightedStateJointHistory(next_one_step_uncompressed_occupancy_state, hidden_state, joint_history);
//                         var.add(IloBoolVar(env, 0, 1, VarName.c_str()));
//                         this->setNumber(VarName, index++);
//                     }
//                 }
//             }
//             this->createDecentralizedVariables(vf,state, env, var, index, t);
//         }
//         catch (const std::exception &exc)
//         {
//             // catch anything thrown within try block that derives from std::exception
//             std::cerr << "SawtoothLPBackup::setGreedyVariables(..) exception caught: " << exc.what() << std::endl;
//             exit(-1);
//         }
//     }

//     double ActionVFSawtoothLP::getQValueRelaxation(const std::shared_ptr<ValueFunction>&vf,const  std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> joint_history, std::shared_ptr<Action> action, number t)
//     {
//         // \sum_{o} a(u|o) \sum_{x} s(x,o) * Q_MDP(x,u)

//         auto weight = 0.0;
//         try
//         {
//             auto compressed_occupancy_state = state->toOccupancyState();
//             auto relaxation = std::static_pointer_cast<RelaxedValueFunction>(vf->getInitFunction());

//             if (relaxation->isPomdpAvailable())
//             {
//                 auto belief = compressed_occupancy_state->createBeliefWeighted(joint_history);
//                 weight =  compressed_occupancy_state->getProbabilityOverJointHistory(joint_history) * relaxation->operator()(std::make_pair(belief, action), t);
//             }
//             else
//             {
//                 for (const auto &x : compressed_occupancy_state->getStatesAt(joint_history))
//                 {
//                     // \sum_{x} s(x,o) * Q_MDP(x,u)
//                     weight += compressed_occupancy_state->getProbability(compressed_occupancy_state->HiddenStateAndJointHistoryToState(x, joint_history)) * relaxation->operator()(std::make_pair(x, action), t);
//                 }
//             }
//         }
//         catch(const std::exception &exc)
//         {
//             // catch anything thrown within try block that derives from std::exception
//             std::cerr << "SawtoothLPBackup::getQValueRelaxation(..) exception caught: " << exc.what() << std::endl;
//             exit(-1);
//         }
//         return weight;
//     }
    
//     void ActionVFSawtoothLP::createConstraints(const std::shared_ptr<ValueFunction>&vf,const std::shared_ptr<State>&state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
//     {
//         assert(vf->getInitFunction() != nullptr);

//         //<!  Build sawtooth constraints v - \sum_{u} a(u|o) * Q(k,s,o,u,y,z, diff, t  ) + \omega_k(y,<o,z>)*M <= M,  \forall k, y,<o,z>
//         //<!  Build sawtooth constraints  Q(k,s,o,u,y,z, diff, t ) = (v_k - V_k) \frac{\sum_{x} s(x,o) * p(x,u,z,y)}}{s_k(y,<o,z>)},  \forall a(u|o)

//        try{
//             this->createInitialConstrainte(vf,state, env, con, var, index, t);

//             number recover = 0;

//             auto compressed_occupancy_state = state->toOccupancyState();

//             // Go over all points in the point set at t+1
//             for (const auto &element_state_AND_upper_bound : this->tmp_representation)
//             {
//                 const auto &next_one_step_uncompressed_occupancy_state = element_state_AND_upper_bound.first;
//                 auto current_upper_bound = element_state_AND_upper_bound.second;

//                 // Compute the difference i.e. (v_k - V_k)
//                 auto initial_upper_bound = vf->getInitFunction()->operator()(next_one_step_uncompressed_occupancy_state, t + 1);
//                 auto difference = current_upper_bound - initial_upper_bound;

//                 // Go over all joint histories in over the support of next_one_step_uncompressed_occupancy_state
//                 for (const auto &next_joint_history : next_one_step_uncompressed_occupancy_state->getJointHistories())
//                 {
//                     for(const auto &next_hidden_state : next_one_step_uncompressed_occupancy_state->getStatesAt(next_joint_history))
//                     {
//                         //Get probability
//                         auto probability = next_one_step_uncompressed_occupancy_state->getProbability(next_one_step_uncompressed_occupancy_state->HiddenStateAndJointHistoryToState(next_hidden_state,next_joint_history));

//                         std::shared_ptr<Joint<std::shared_ptr<Observation>>> next_joint_observation;
//                         // Get next observation
//                         if (compressed_occupancy_state->getTypeState() == TypeState::SERIAL_OCCUPANCY_STATE)
//                         {
//                             int next_agent_id = compressed_occupancy_state->toSerialOccupancyState()->getCurrentAgentId();
//                             next_joint_observation = next_agent_id == 0 ? next_joint_history->getData() : next_joint_history->getDefaultObs();
//                         }
//                         else
//                         {
//                             next_joint_observation = next_joint_history->getData();
//                         }

//                         // Verification of joint_history 
//                         for(const auto &joint_history : compressed_occupancy_state->getJointHistories())
//                         {
//                             auto verification = joint_history->expand(next_joint_observation);
//                             if(verification == next_joint_history)
//                             {
//                                 switch (this->current_type_of_resolution_)
//                                 {
//                                 case TypeOfResolution::BigM:
//                                     // this->createSawtoothBigM(compressed_occupancy_state,joint_history,next_hidden_state,next_observation,next_joint_history,next_one_step_uncompressed_occupancy_state,probability,difference,env,con,var,index, t);
//                                     break;
//                                 case TypeOfResolution::IloIfThenResolution:
//                                     // this->createSawtoothIloIfThen(compressed_occupancy_state,joint_history,next_hidden_state,next_observation,next_joint_history,next_one_step_uncompressed_occupancy_state,probability,difference,env,model,var, t);
//                                     break;
//                                 }
//                             }
//                         }
//                     }
//                 }

//                 // Build constraint \sum{x',o'} \omega_k(x',o') = 1
//                 con.add(IloRange(env, 1.0, 1.0));

//                 // Go over all joint histories in over the support of next_one_step_uncompressed_occupancy_state
//                 for (const auto &next_joint_history : next_one_step_uncompressed_occupancy_state->getJointHistories())
//                 {
//                     for(const auto &next_hidden_state : next_one_step_uncompressed_occupancy_state->getStatesAt(next_joint_history))
//                     {
//                         // <! \omega_k(x',o')
//                         auto VarName = this->getVarNameWeightedStateJointHistory(next_one_step_uncompressed_occupancy_state, next_hidden_state, next_joint_history);

//                         recover = this->getNumber(VarName);
//                         con[index].setLinearCoef(var[recover], +1.0);
//                     }
//                 }
//                 index++;
//             }
//         }
//         catch (const std::exception &exc)
//         {
//             // catch anything thrown within try block that derives from std::exception
//             std::cerr << "ActionVFSawtoothLP::setGreedySawtooth(..) exception caught: " << exc.what() << std::endl;
//             exit(-1);
//         }
//     }

//     double ActionVFSawtoothLP::getQValueRealistic(const std::shared_ptr<ValueFunction>&vf,const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface>& joint_history, const std::shared_ptr<Action>& action, std::shared_ptr<State> next_hidden_state, const std::shared_ptr<Observation> next_observation, double denominator, double difference, number t)
//     {
//         return difference * this->getSawtoothMinimumRatio(vf,state, joint_history, action, next_hidden_state, next_observation, denominator,t);
//     }
    
//     double ActionVFSawtoothLP::getSawtoothMinimumRatio(const std::shared_ptr<ValueFunction>&,const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface>& joint_history, const std::shared_ptr<Action>& action, const std::shared_ptr<State>& next_hidden_state, const std::shared_ptr<Observation>& next_observation, double denominator, number t)
//     {
//         auto factor = 0.0;

//         try
//         {
//             auto compressed_occupancy_state = state->toOccupancyState();
//             auto under_pb = std::dynamic_pointer_cast<MPOMDPInterface>(ActionVFBase<double>::world_->getUnderlyingProblem());

//             // Go over all state conditionning to a joint history
//             for (const auto &hidden_state : compressed_occupancy_state->getStatesAt(joint_history))
//             {
//                 // \sum_{x} s(x,o) * p_{x,u,z',x'}
//                 factor += compressed_occupancy_state->getProbability(compressed_occupancy_state->HiddenStateAndJointHistoryToState(hidden_state, joint_history)) * under_pb->getDynamics(hidden_state, action, next_hidden_state,next_observation,t);
//             }
//         }
//         catch (const std::exception &exc)
//         {
//             // catch anything thrown within try block that derives from std::exception
//             std::cerr << "ActionVFSawtoothLP::getSawtoothMinimumRatio(..) exception caught: " << exc.what() << std::endl;
//             exit(-1);
//         }
//         return factor / denominator;
//     }


    
// }
