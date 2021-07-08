// #include <sdm/utils/value_function/action_vf/action_sawtooth_lp_occupancy.hpp>

// namespace sdm
// {
//     ActionVFSawtoothLPOccupancy::ActionVFSawtoothLPOccupancy();{}
//     ActionVFSawtoothLPOccupancy::ActionVFSawtoothLPOccupancy(const std::shared_ptr<SolvableByHSVI>& world,TypeOfResolution current_type_of_resolution, number bigM_value, TypeSawtoothLinearProgram type_sawtooth_resolution) : ActionVFSawtoothLP(world,current_type_of_resolution,bigM_value,type_sawtooth_resolution)
    
//     void ActionVFSawtoothLPOccupancy::createDecentralizedVariables(const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t)
//     {
//         auto under_pb = this->world_->getUnderlyingProblem();

//         this->createDecentralizedVariablesJoint(state, env, var, index, t);

//         for (auto agent = 0; agent < under_pb->getNumAgents(); ++agent)
//         {
//             this->createDecentralizedVariablesIndividual(state, env, var, index, t,agent);
//         }
//     }

//     void ActionVFSawtoothLPOccupancy::createDecentralizedConstraints(const std::shared_ptr<State>& state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
//     {
//         auto under_pb = this->world_->getUnderlyingProblem();

//         this->createDecentralizedConstraintsJoint(state, env, con, var, index, t);
//         this->createDecentralizedControlConstraints(state, env, con, var, index, t);

//         for (number agent = 0; agent < under_pb->getNumAgents(); ++agent)
//         {
//             this->createDecentralizedConstraintsIndividual(state, env, con, var, index, t,agent);
//         }
//     }

//     std::shared_ptr<Action> ActionVFSawtoothLPOccupancy::getVariableResult(const std::shared_ptr<State> &state,const IloCplex &cplex, const IloNumVarArray &var, number t)
//     {
//         number index = 0;
//         std::vector<std::vector<std::shared_ptr<Item>>> actions;
//         std::vector<std::vector<std::shared_ptr<Item>>> joint_histories;

//         auto under_pb = this->world_->getUnderlyingProblem();

//         for (number agent = 0; agent < under_pb->getNumAgents(); agent++)
//         {
//             actions.push_back({});
//             joint_histories.push_back({});

//             auto action_and_history_individual = this->getVariableResultIndividual(state,cplex,var,t,agent);

//             actions[agent].push_back(action_and_history_individual.first);
//             joint_histories[agent].push_back(action_and_history_individual.second);
//         }
        
//         return std::make_shared<JointDeterministicDecisionRule>(joint_histories,actions);
//     }

//     void ActionVFSawtoothLPOccupancy::createSawtoothBigM(const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface>& joint_history, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double probability, double difference, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
//     {
//         try
//         {
//             auto under_pb = std::dynamic_pointer_cast<MMDPInterface>(this->world_->getUnderlyingProblem());

//             number recover = 0;

//             con.add(IloRange(env, -IloInfinity, this->bigM_value_));
//             con[index].setLinearCoef(var[this->getNumber(this->getVarNameWeight(0))], +1.0);

//             // Go over all actions
//             for (const auto &action : *under_pb->getActionSpace(t))
//             {
//                 //<! 1.c.4 get variable a(u|o) and set constant
//                 recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action->toAction(), joint_history));
//                 con[index].setLinearCoef(var[recover], - this->getQValueRealistic(state, joint_history, action->toAction(), next_hidden_state, next_observation, probability, difference));
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

//     void ActionVFSawtoothLPOccupancy::setGreedySawtoothIloIfThen(const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface>& joint_history, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double probability, double difference, IloEnv &env, IloModel &model, IloNumVarArray &var,number t)
//     {
//         try
//         {
//             number recover = 0;

//             auto under_pb = std::dynamic_pointer_cast<MMDPInterface>(this->world_->getUnderlyingProblem());

//             IloExpr expr(env);
//             //<! 1.c.1 get variable v and set coefficient of variable v
//             expr = var[this->getNumber(this->getVarNameWeight(0))];

//             // Go over all actions
//             for(const auto & action : *under_pb->getActionSpace(t))
//             { 
//                 recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action->toAction(), joint_history));
//                 //<! 1.c.4 get variable a(u|o) and set constant 
//                 expr -= this->getQValueRealistic(compressed_occupancy_state, joint_history, action, next_hidden_state, next_observation, probability, difference) * var[recover];
//             }               

//             // <! get variable \omega_k(x',o')
//             recover = this->getNumber(this->getVarNameWeightedStateJointHistory(next_one_step_uncompressed_occupancy_state, next_hidden_state, next_joint_history));
//             model.add(IloIfThen(env, var[recover] > 0, expr <= 0));
//         }
//         catch (const std::exception &exc)
//         {
//             // catch anything thrown within try block that derives from std::exception
//             std::cerr << "SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedySawtoothIloIfThen(..) exception caught: " << exc.what() << std::endl;
//             exit(-1);
//         }
//     }
// }