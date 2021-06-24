// #include <sdm/utils/value_function/backup/sawtooth_lp_backup.hpp>

// namespace sdm
// {
//     SawtoothLPBackup::SawtoothLPBackup() {}

//     SawtoothLPBackup::SawtoothLPBackup(const std::shared_ptr<SolvableByHSVI>& world,TypeOfResolution current_type_of_resolution, number bigM_value, TypeSawtoothLinearProgram type_sawtooth_resolution) : 
//         SawtoothBackup(world) , csawtooth_lp_(type_sawtooth_resolution), current_type_of_resolution_(current_type_of_resolution)

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

//     double SawtoothLPBackup::backup(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t);
//     {
//         // double cub = 0;
//         // this->getGreedy(compressed_occupancy_state, cub, t);
//         // return cub;
//     }

//     std::shared_ptr<Action> SawtoothLPBackup::getBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t)
//     {
//         // double cub = 0;
//         // return this->getGreedy(compressed_occupancy_state, cub, t);
//     }

//     Pair<std::shared_ptr<Action>,double> greedy(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<OccupancyStateInterface> &hyperplan, number t)
//     {
//         switch (this->getSawtoothType())
//         {
//         case TypeSawtoothLinearProgram::PLAIN_SAWTOOTH_LINER_PROGRAMMING:
//             return this->greedyFullSawtooth(vf,occupancy_state, t);
//         case TypeSawtoothLinearProgram::RELAXED_SAWTOOTH_LINER_PROGRAMMING:
//             return this->greedyRelaxedSawtooth(vf,occupancy_state, t);
//         default:
//             return this->greedyFullSawtooth(vf,occupancy_state, t);
//         }
//     }

//     Pair<std::shared_ptr<Action>,double> SawtoothLPBackup::greedyRelaxedSawtooth(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<OccupancyStateInterface> &occupancy_state, number t)
//     {
//         std::shared_ptr<Action> greedy, action;
//         double value;
//         cub = std::numeric_limits<double>::max();

//         if (vf->getSupport(t + 1).empty())
//         {
//             this->tmp_representation = {};
//             greedy = this->getGreedy(occupancy_state, t).first;
//         }

//         else
//         {
//             for (const auto &element : vf->getSupport(t + 1))
//             {
//                 this->tmp_representation = {element};

//                 action = this->getGreedy(occupancy_state, t);

//                 value = this->getQValueAt(occupancy_state, action, t);

//                 //if( this->representation[t+1].size() > 1 ) std::cout << "\thorizon=" << t << "\tvalue=" << value << "\t cub=" << cub << "\t vub=" << vub << std::endl;

//                 if (cub > value)
//                 {
//                     cub = value;
//                     greedy = action;
//                 }
//             }
//         }

//         return greedy;
//     }

//     Pair<std::shared_ptr<Action>,double> SawtoothLPBackup::greedyFullSawtooth(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<OccupancyStateInterface>  &occupancy_state, number t)
//     {
//         this->tmp_representation = this->representation[t + 1];
//         return this->greedyActionSelectionBySawtooth(occupancy_state, cub, t);
//     }

//     Pair<std::shared_ptr<Action>,double> SawtoothLPBackup::getGreedy(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, number t)
//     {
//         number index = 0;

//         std::shared_ptr<Action> action;
//         double value;

//         IloEnv env;
//         try
//         {
//             IloModel model(env);

//             // Init the model
//             IloRangeArray con(env);
//             IloNumVarArray var(env);

//             IloObjective obj = IloMaximize(env);

//             ///////  BEGIN CORE CPLEX Code  ///////

//             // 0. Build variables a(u|o), a_i(u_i|o_i), v
//             this->setGreedyVariables(occupancy_state, env, var, t);

//             // 1. Build objective function \sum_{o,u} a(u|o) \sum_x s(x,o) Q_MDP(x,u) - discount * v
//             this->setGreedyObjective(occupancy_state, obj, var, t);

//             //<! 3.a Build sawtooth constraints v <= (V_k - v_k) * \frac{\sum_{o,u} a(u|o)\sum_{x,z_} s(x,o)*p(x,u,z_,x_)}}{s_k(x_,o_)} ,\forall k, x_,o_
//             this->setGreedySawtooth(occupancy_state, model, env, con, var, index, t);

//             // 3. Build decentralized control constraints [  a(u|o) >= \sum_i a_i(u_i|o_i) + 1 - n ] ---- and ---- [ a(u|o) <= a_i(u_i|o_i) ]
//             this->setDecentralizedConstraints(occupancy_state, env, con, var, index, t);

//             ///////  END CORE  CPLEX Code ///////
//             model.add(obj);
//             model.add(con);
//             IloCplex cplex(model);
//             cplex.setOut(env.getNullStream());
//             cplex.setWarning(env.getNullStream());

//             // Optimize the problem and obtain solution
//             cplex.exportModel("bellman_greedy_op.lp");
//             if (!cplex.solve())
//             {
//                 env.error() << "Failed to optimize MILP" << std::endl;
//                 system("cat bellman_greedy_op.lp");
//                 throw(-1);
//             }
//             else
//             {
//                 value = cplex.getObjValue();
//                 // action = this->getDecentralizedVariables(cplex, var, occupancy_state, t);
//             }
//         }
//         catch (IloException &e)
//         {
//             std::cerr << "Concert exception caught: " << e << std::endl;
//             exit(-1);
//         }
//         catch (const std::exception &exc)
//         {
//             // catch anything thrown within try block that derives from std::exception
//             std::cerr << "Non-Concert exception caught: " << exc.what() << std::endl;
//             exit(-1);
//         }

//         env.end();

//         return std::make_pair(action,value);
//     }

//     void SawtoothLPBackup::setGreedyVariables(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, IloEnv &env, IloNumVarArray &var, number t)
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
//                         auto hidden_state = next_one_step_uncompressed_occupancy_state->getState(hidden_state_AND_joint_history);
//                         auto joint_history = next_one_step_uncompressed_occupancy_state->getHistory(hidden_state_AND_joint_history);

//                         // <! \omega_k(x',o')
//                         VarName = this->getVarNameWeightedStateJointHistory(next_one_step_uncompressed_occupancy_state, hidden_state, joint_history);
//                         var.add(IloBoolVar(env, 0, 1, VarName.c_str()));
//                         this->setNumber(VarName, index++);
//                     }
//                 }
//             }
//             this->setDecentralizedVariables(occupancy_state, env, var, index, t);
//         }
//         catch (const std::exception &exc)
//         {
//             // catch anything thrown within try block that derives from std::exception
//             std::cerr << "SawtoothLPBackup::setGreedyVariables(..) exception caught: " << exc.what() << std::endl;
//             exit(-1);
//         }
//     }

//     void SawtoothLPBackup::setGreedyObjective(const std::shared_ptr<OccupancyStateInterface> &compressed_serial_occupancy_state, IloObjective &obj, IloNumVarArray &var, number t)
//     {
//         switch (compressed_serial_occupancy_state->getTypeState())
//         {
//         case TypeState::OCCUPANCY_STATE :
//             this->setGreedyObjectiveOccupancy(compressed_serial_occupancy_state,obj,var,t);
//             break;
//         case TypeState::SERIAL_OCCUPANCY_STATE :
//             this->setGreedyObjectiveSerialOccupancy(compressed_serial_occupancy_state,obj,var,t);
//             break;
        
//         default:
//             exit(-1);
//             break;
//         }
//     }

//     void SawtoothLPBackup::setGreedyObjectiveOccupancy(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, IloNumVarArray &var, IloObjective &obj, number t)
//     {
//         try
//         {
//             auto under_pb = std::dynamic_pointer_cast<MPOMDPInterface>(MaxPlanBackup::world_->getUnderlyingProblem());

//             // <! 1.a get variable v
//             auto recover = this->getNumber(this->getVarNameWeight(0));

//             //<! 1.b set coefficient of objective function "\sum_{o,u} a(u|o) \sum_x s(x,o) Q_MDP(x,u) - discount * v0"
//             obj.setLinearCoef(var[recover], under_pb->getDiscount(t));

//             // Go over all action
//             for (const auto &action : under_pb->getActionSpace(t))
//             {
//                 // Go over all joint history
//                 for (const auto &joint_history : compressed_occupancy_state.getJointHistories())
//                 {
//                     //<! 1.c.4 get variable a(u|o)
//                     recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action, joint_history));

//                     //<! 1.c.5 set coefficient of variable a(u|o) i.e., \sum_x s(x,o) Q_MDP(x,u)
//                     obj.setLinearCoef(var[recover], this->getQValueRelaxation(compressed_occupancy_state, joint_history, action, t));
//                 }
//             }
//         }
//         catch (const std::exception &exc)
//         {
//             // catch anything thrown within try block that derives from std::exception
//             std::cerr << "SawtoothLPBackup::setGreedyObjective(..) exception caught: " << exc.what() << std::endl;
//             exit(-1);
//         }
//     }

//     void SawtoothLPBackup::setGreedyObjectiveSerialOccupancy(const std::shared_ptr<OccupancyStateInterface> &compressed_serial_occupancy_state, IloObjective &obj, IloNumVarArray &var, number t)
//     {
//         auto under_pb = std::dynamic_pointer_cast<SerializedMPOMDP>(MaxPlanBackup::world_->getUnderlyingProblem());
//         number agent_id = under_pb->getAgentId(t);

//         // <! 1.a get variable v
//         auto recover = this->getNumber(this->getVarNameWeight(0));

//         //<! 1.b set coefficient of objective function "\sum_{o_i,u_i} a_i(u_i|o_i) \sum_x s(x,o_i) Q_MDP(x,u_i) + discount * v0"
//         obj.setLinearCoef(var[recover], under_pb->getDiscount(t));

//         // Go over all joint history
//         for (const auto &indiv_history : compressed_serial_occupancy_state->getIndividualHistories(agent_id))
//         {
//             for (const auto serial_action : *under_pb->getActionSpace(t))
//             {
//                 recover = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(serial_action, indiv_history, agent_id));

//                 double res = 0;
//                 for (const auto &joint_history : compressed_serial_occupancy_state->getJointHistories())
//                 {
//                     if (joint_history->getIndividualHistory(agent_id) == indiv_history)
//                     {
//                         //<! 1.c.5 set coefficient of variable a_i(u_i|o_i) i.e., \sum_x s(x,o_i) Q_MDP(x,u_i)
//                         res += this->getQValueRelaxation(compressed_serial_occupancy_state, joint_history, serial_action, t);
//                     }
//                 }
//                 obj.setLinearCoef(var[recover], res);
//             }
//         }
//     }

//     // double SawtoothLPBackup::getQValueRealistic(const TState &compressed_occupancy_state, typename TState::jhistory_type joint_history, typename TAction::output_type action, typename TState::state_type next_hidden_state, typename TState::observation_type next_observation, double denominator, double difference)
//     // {
//     //     return difference * this->template getSawtoothMinimumRatio<TState>(compressed_occupancy_state, joint_history, action, next_hidden_state, next_observation, denominator);
//     // }

//     double SawtoothLPBackup::getQValueRelaxation(const  std::shared_ptr<OccupancyStateInterface> &compressed_occupancy_state, const std::shared_ptr<JointHistoryInterface> joint_history, std::shared_ptr<Action> action, number t)
//     {
//         // \sum_{o} a(u|o) \sum_{x} s(x,o) * Q_MDP(x,u)

//         auto weight = 0.0;
//         try
//         {
//             auto relaxation = std::static_pointer_cast<RelaxedValueFunction>(this->getInitFunction());

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
//                     weight += compressed_occupancy_state->getProbability(compressed_occupancy_state->HiddenStateAndJointHistoryToState(x, joint_history) * relaxation->operator()(std::make_pair(x, action), t);
//                 }
//             }
//         }
//         catch (const std::exception &exc)
//         {
//             // catch anything thrown within try block that derives from std::exception
//             std::cerr << "SawtoothLPBackup::getQValueRelaxation(..) exception caught: " << exc.what() << std::endl;
//             exit(-1);
//         }
//         return weight;
//     }

//     void SawtoothLPBackup::setGreedySawtooth(const std::shared_ptr<OccupancyStateInterface> &compressed_occupancy_state, IloModel &model, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
//     {

//         assert(this->getInitFunction() != nullptr);

//         // this->template setInitialConstrainte<TState>(compressed_occupancy_state, env, con, var, index, t);

//         number recover = 0;

//         //<!  Build sawtooth constraints v - \sum_{u} a(u|o) * Q(k,s,o,u,y,z, diff, t  ) + \omega_k(y,<o,z>)*M <= M,  \forall k, y,<o,z>
//         //<!  Build sawtooth constraints  Q(k,s,o,u,y,z, diff, t ) = (v_k - V_k) \frac{\sum_{x} s(x,o) * p(x,u,z,y)}}{s_k(y,<o,z>)},  \forall a(u|o)

//        try{
//             // Go over all points in the point set at t+1
//             for (const auto &element_state_AND_upper_bound : this->tmp_representation)
//             {
//                 const auto &next_one_step_uncompressed_occupancy_state = element_state_AND_upper_bound.first;
//                 auto current_upper_bound = element_state_AND_upper_bound.second;

//                 // Compute the difference i.e. (v_k - V_k)
//                 auto initial_upper_bound = this->getInitFunction()->operator()(next_one_step_uncompressed_occupancy_state, t + 1);
//                 auto difference = current_upper_bound - initial_upper_bound;

//                 // Go over all joint histories in over the support of next_one_step_uncompressed_occupancy_state
//                 for (const auto &next_joint_history : next_one_step_uncompressed_occupancy_state->getJointHistories())
//                 {
//                     for(const auto &next_hidden_state : next_one_step_uncompressed_occupancy_state->getStatesAt(joint_history))
//                     {
//                         //Get probability
//                         auto probability = next_one_step_uncompressed_occupancy_state->getProbability(next_one_step_uncompressed_occupancy_state->HiddenStateAndJointHistoryToState(next_hidden_state,next_joint_history));

//                         std::shared_ptr<Observation> next_observation;
//                         // Get next observation
//                         if (compressed_occupancy_state->getTypeState() == TypeState::SERIAL_OCCUPANCY_STATE)
//                         {
//                             int next_agent_id = compressed_occupancy_state->toSerialOccupancyState()->getCurrentAgentId();
//                             next_observation = next_agent_id == 0 ? next_joint_history->getData() : next_joint_history->getDefaultObs();
//                         }
//                         else
//                         {
//                             next_observation = next_joint_history->getData();
//                         }

//                         // Verification of joint_history 
//                         for(const auto &joint_history : compressed_occupancy_state.getJointHistories())
//                         {
//                             auto verification = joint_history->expand(next_observation);
//                             if(verification == next_joint_history)
//                             {
//                                 switch (this->current_type_of_resolution_)
//                                 {
//                                 case TypeOfResolution::BigM:
//                                     // this->template setGreedySawtoothBigM<TState>(compressed_occupancy_state,joint_history,next_hidden_state,next_observation,next_joint_history,next_one_step_uncompressed_occupancy_state,probability,difference,env,con,var,index, t);
//                                     break;
//                                 case TypeOfResolution::IloIfThenResolution:
//                                     // this->template setGreedySawtoothIloIfThen<TState>(compressed_occupancy_state,joint_history,next_hidden_state,next_observation,next_joint_history,next_one_step_uncompressed_occupancy_state,probability,difference,env,model,var, t);
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
//                     for(const auto &next_hidden_state : next_one_step_uncompressed_occupancy_state->getStatesAt(joint_history))
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
//             std::cerr << "SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedySawtooth(..) exception caught: " << exc.what() << std::endl;
//             exit(-1);
//         }
//     }

//     // template <typename T, std::enable_if_t<std::is_any<T, OccupancyState<>, OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>>::value, int>>
//     // void SawtoothLPBackup::setGreedySawtoothBigM(const TState &compressed_occupancy_state, typename TState::jhistory_type &joint_history, typename TState::state_type &next_hidden_state, typename TState::observation_type &next_observation, typename TState::jhistory_type &next_joint_history, const TState &next_one_step_uncompressed_occupancy_state, double probability, double difference, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number)
//     // {
//     //     try
//     //     {
//     //         con.add(IloRange(env, -IloInfinity, this->bigM_value_));
//     //         con[index].setLinearCoef(var[this->getNumber(this->getVarNameWeight(0))], +1.0);

//     //         if (compressed_occupancy_state.getJointHistories().find(joint_history) != compressed_occupancy_state.getJointHistories().end())
//     //         {
//     //             // Go over all actions
//     //             for (const auto &action : this->getWorld()->getUnderlyingProblem()->getActionSpace()->getAll())
//     //             {
//     //                 //<! 1.c.4 get variable a(u|o) and set constant
//     //                 con[index].setLinearCoef(var[this->getNumber(this->getVarNameJointHistoryDecisionRule(action, joint_history))], -this->getQValueRealistic(compressed_occupancy_state, joint_history, action, next_hidden_state, next_observation, probability, difference));
//     //             }
//     //         }

//     //         // <! \omega_k(x',o') * BigM
//     //         auto VarName = this->getVarNameWeightedStateJointHistory(next_one_step_uncompressed_occupancy_state, next_hidden_state, next_joint_history);
//     //         con[index].setLinearCoef(var[this->getNumber(VarName)], this->bigM_value_);

//     //         index++;
//     //     }
//     //     catch (const std::exception &exc)
//     //     {
//     //         // catch anything thrown within try block that derives from std::exception
//     //         std::cerr << "SawtoothLPBackup::setGreedySawtoothBigM(..) exception caught: " << exc.what() << std::endl;
//     //         exit(-1);
//     //     }
//     // }

//     // template <typename T, std::enable_if_t<std::is_any<T, OccupancyState<>, OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>>::value, int>>
//     // void SawtoothLPBackup::setGreedySawtoothIloIfThen(const TState &compressed_occupancy_state, typename TState::jhistory_type &joint_history, typename TState::state_type &next_hidden_state, typename TState::observation_type &next_observation, typename TState::jhistory_type &next_joint_history, const TState &next_one_step_uncompressed_occupancy_state, double probability, double difference, IloEnv &env, IloModel &model, IloNumVarArray &var, number)
//     // {
//     //     number recover = 0;

//     //     try
//     //     {
//     //         IloExpr expr(env);
//     //         //<! 1.c.1 get variable v and set coefficient of variable v
//     //         expr = var[this->getNumber(this->getVarNameWeight(0))];

//     //         if (compressed_occupancy_state.getJointHistories().find(joint_history) != compressed_occupancy_state.getJointHistories().end())
//     //         {
//     //             // Go over all actions
//     //             for (const auto &action : this->getWorld()->getUnderlyingProblem()->getActionSpace()->getAll())
//     //             {
//     //                 //<! 1.c.4 get variable a(u|o) and set constant
//     //                 expr -= this->getQValueRealistic(compressed_occupancy_state, joint_history, action, next_hidden_state, next_observation, probability, difference) * var[this->getNumber(this->getVarNameJointHistoryDecisionRule(action, joint_history))];
//     //             }
//     //         }

//     //         // <! get variable \omega_k(x',o')
//     //         recover = this->getNumber(this->getVarNameWeightedStateJointHistory(next_one_step_uncompressed_occupancy_state, next_hidden_state, next_joint_history));
//     //         model.add(IloIfThen(env, var[recover] > 0, expr <= 0));
//     //     }
//     //     catch (const std::exception &exc)
//     //     {
//     //         // catch anything thrown within try block that derives from std::exception
//     //         std::cerr << "SawtoothLPBackup::setGreedySawtoothIloIfThen(..) exception caught: " << exc.what() << std::endl;
//     //         exit(-1);
//     //     }
//     // }

//     // template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<>, T>, int>>
//     // double SawtoothLPBackup::getSawtoothMinimumRatio(const TState &compressed_occupancy_state, typename TState::jhistory_type joint_history, typename TAction::output_type action, typename TState::state_type next_hidden_state, typename TState::observation_type next_observation, double denominator)
//     // {
//     //     auto factor = 0.0;

//     //     try
//     //     {
//     //         // Go over all state conditionning to a joint history
//     //         for (const auto &hidden_state : compressed_occupancy_state.getStatesAt(joint_history))
//     //         {
//     //             // \sum_{x} s(x,o) * p_{x,u,z',x'}
//     //             factor += compressed_occupancy_state.at(std::make_pair(hidden_state, joint_history)) * this->getWorld()->getUnderlyingProblem()->getObsDynamics()->getDynamics(hidden_state, this->getWorld()->getUnderlyingProblem()->getActionSpace()->joint2single(action), this->getWorld()->getUnderlyingProblem()->getObsSpace()->joint2single(next_observation), next_hidden_state);
//     //         }
//     //     }
//     //     catch (const std::exception &exc)
//     //     {
//     //         // catch anything thrown within try block that derives from std::exception
//     //         std::cerr << "SawtoothLPBackup::getSawtoothMinimumRatio(..) exception caught: " << exc.what() << std::endl;
//     //         exit(-1);
//     //     }

//     //     return factor / denominator;
//     // }

//     // // --------------------------------------------------------------------------
//     // // -------------  SerializedOccupancyState<TState, JointHistory>  ----------------
//     // // --------------------------------------------------------------------------


//     // template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int>>
//     // double SawtoothLPBackup::getSawtoothMinimumRatio(const TState &compressed_serial_occupancy_state, typename TState::jhistory_type joint_history, typename TAction::output_type action, typename TState::state_type next_hidden_serial_state, typename TState::observation_type next_observation, double denominator)
//     // {
//     //     double factor = 0.0;

//     //     // Go over all hidden serial state  conditional to a joint_history
//     //     for (const auto &hidden_serial_state : compressed_serial_occupancy_state.getStatesAt(joint_history))
//     //     {
//     //         // \sum_{x} s(x,o) * p_{x,u,z',x'}
//     //         factor += compressed_serial_occupancy_state.at(std::make_pair(hidden_serial_state, joint_history)) * this->getWorld()->getUnderlyingProblem()->getDynamics(hidden_serial_state, action, next_observation, next_hidden_serial_state);
//     //     }

//     //     return factor / denominator;
//     // }

//     // template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int>>
//     // void SawtoothLPBackup::setGreedySawtoothBigM(const TState &compressed_serial_occupancy_state, typename TState::jhistory_type &joint_history, typename TState::state_type &next_hidden_state, typename TState::observation_type &next_observation, typename TState::jhistory_type &next_joint_history, const TState &next_one_step_uncompressed_serial_occupancy_state, double probability, double difference, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
//     // {

//     //     number agent_id = compressed_serial_occupancy_state.getCurrentAgentId();
//     //     // Gets the current individual history conditional on the current joint history
//     //     auto indiv_history = joint_history->getIndividualHistory(agent_id);

//     //     con.add(IloRange(env, -IloInfinity, this->bigM_value_));
//     //     con[index].setLinearCoef(var[this->getNumber(this->getVarNameWeight(0))], +1.0);

//     //     if (compressed_serial_occupancy_state.getJointHistories().find(joint_history) != compressed_serial_occupancy_state.getJointHistories().end())
//     //     {
//     //         // Go over all actions
//     //         for (const auto &serial_action : this->getWorld()->getUnderlyingProblem()->getActionSpace(t)->getAll())
//     //         {
//     //             //<! 1.c.4 get variable a(u|o) and set constant
//     //             con[index].setLinearCoef(var[this->getNumber(this->getVarNameIndividualHistoryDecisionRule(serial_action, indiv_history, agent_id))], -this->getQValueRealistic(compressed_serial_occupancy_state, joint_history, serial_action, next_hidden_state, next_observation, probability, difference));
//     //         }
//     //     }
//     //     // <! \omega_k(x',o') * BigM
//     //     auto VarName = this->getVarNameWeightedStateJointHistory(next_one_step_uncompressed_serial_occupancy_state, next_hidden_state, next_joint_history);
//     //     con[index].setLinearCoef(var[this->getNumber(VarName)], this->bigM_value_);

//     //     index++;
//     // }

//     // template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int>>
//     // void SawtoothLPBackup::setGreedySawtoothIloIfThen(const TState &compressed_serial_occupancy_state, typename TState::jhistory_type &joint_history, typename TState::state_type &next_hidden_state, typename TState::observation_type &next_observation, typename TState::jhistory_type &next_joint_history, const TState &next_one_step_uncompressed_serial_occupancy_state, double probability, double difference, IloEnv &env, IloModel &model, IloNumVarArray &var, number t)
//     // {
//     //     number agent_id = compressed_serial_occupancy_state.getCurrentAgentId();
//     //     // Gets the current individual history conditional on the current joint history
//     //     auto indiv_history = joint_history->getIndividualHistory(agent_id);

//     //     number recover = 0;

//     //     IloExpr expr(env);
//     //     //<! 1.c.1 get variable v and set coefficient of variable v
//     //     expr = var[this->getNumber(this->getVarNameWeight(0))];

//     //     if (compressed_serial_occupancy_state.getJointHistories().find(joint_history) != compressed_serial_occupancy_state.getJointHistories().end())
//     //     {
//     //         // Go over all actions
//     //         for (const auto &serial_action : this->getWorld()->getUnderlyingProblem()->getActionSpace(t)->getAll())
//     //         {
//     //             //<! 1.c.4 get variable a(u|o) and set constant
//     //             expr -= this->getQValueRealistic(compressed_serial_occupancy_state, joint_history, serial_action, next_hidden_state, next_observation, probability, difference) * var[this->getNumber(this->getVarNameIndividualHistoryDecisionRule(serial_action, indiv_history, agent_id))];
//     //         }
//     //     }

//     //     // <! get variable \omega_k(x',o')
//     //     recover = this->getNumber(this->getVarNameWeightedStateJointHistory(next_one_step_uncompressed_serial_occupancy_state, next_hidden_state, next_joint_history));
//     //     model.add(IloIfThen(env, var[recover] > 0, expr <= 0));
//     // }

//     // // --------------------------------------------------------------------------
//     // // -------------  OccupancyState<BeliefState, JointHistory>  ----------------
//     // // --------------------------------------------------------------------------

//     // template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>, T>, int>>
//     // double SawtoothLPBackup::getSawtoothMinimumRatio(const TState &compressed_occupancy_state,
//     //                                                                                  typename TState::jhistory_type joint_history,
//     //                                                                                  typename TAction::output_type action,
//     //                                                                                  typename TState::state_type next_belief,
//     //                                                                                  typename TState::observation_type next_observation,
//     //                                                                                  double denominator)
//     // {
//     //     assert(compressed_occupancy_state.getStatesAt(joint_history).size() == 1);

//     //     auto under_pb = this->getWorld()->getUnderlyingProblem();
//     //     auto factor = 0.0;

//     //     for (const auto &belief : compressed_occupancy_state.getStatesAt(joint_history))
//     //     {
//     //         // Gets next belief b' = T(b,u,z)
//     //         auto next_computed_belief = belief->expand(under_pb->getActionSpace()->joint2single(action),
//     //                                                    under_pb->getObsSpace()->joint2single(next_observation));

//     //         // If next computed beliefs is equal to input next belief
//     //         if (next_computed_belief == next_belief)
//     //         {
//     //             // factor += s_t(b_t,o_t) * p(b_{t+1} | b_t, a)
//     //             factor += compressed_occupancy_state.at(std::make_pair(belief, joint_history)) * belief->getProbability(under_pb->getActionSpace()->joint2single(action),
//     //                                                                                                                     under_pb->getObsSpace()->joint2single(next_observation));
//     //         }
//     //     }

//     //     return (factor / denominator);
//     // }

//     // template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>, T>, int>>
//     // double SawtoothLPBackup::getQValueRelaxation(const TState &compressed_occupancy_state, typename TState::jhistory_type joint_history, typename TAction::output_type action, number t)
//     // {
//     //     // assert(compressed_occupancy_state.getStatesAt(joint_history).size() == 1);
//     //     auto weight = 0.0;
//     //     // for (const auto &belief : compressed_occupancy_state.getStatesAt(joint_history))
//     //     // {
//     //     //     for (number state = 0; state < belief->getData().size(); ++state)
//     //     //     {
//     //     //         weight += compressed_occupancy_state.at(std::make_pair(belief, joint_history)) * belief->getData()[state] * this->getInitFunction()->operator()(std::make_pair(state, this->getWorld()->getUnderlyingProblem()->getActionSpace()->joint2single(action)), t);
//     //     //     }
//     //     // }
//     //     return weight;
//     // }
// }
