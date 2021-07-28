#include <sdm/utils/value_function/action_vf/action_sawtooth_lp.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>
#include <sdm/core/state/interface/serial_interface.hpp>

#include <sdm/world/base/mpomdp_interface.hpp>
#include <sdm/world/occupancy_mdp.hpp>

namespace sdm
{
    ActionVFSawtoothLP::ActionVFSawtoothLP() {}
    ActionVFSawtoothLP::ActionVFSawtoothLP(const std::shared_ptr<SolvableByHSVI>& world,TypeOfResolution current_type_of_resolution, number bigM_value, TypeSawtoothLinearProgram type_of_linear_program) : 
        ActionVFBase(world),DecentralizedLP(world) , current_type_of_resolution_(current_type_of_resolution), type_of_linear_program_(type_of_linear_program)
    {
        this->bigM_value_ = bigM_value;
    }

    Pair<std::shared_ptr<Action>, double> ActionVFSawtoothLP::selectBestActionFull(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state,number t)
    {
        // For the Full version of Sawtooth, wo over all the Point Set  
        this->all_support = std::unordered_map<std::shared_ptr<State>,std::unordered_map<std::shared_ptr<HistoryInterface>,std::vector<std::shared_ptr<State>>>>();

        for (const auto &ostate : vf->getSupport(t+1))
        {
            const auto &next_one_step_uncompressed_occupancy_state = ostate->toOccupancyState()->getOneStepUncompressedOccupancy();

            this->all_support.emplace(ostate, std::unordered_map<std::shared_ptr<HistoryInterface>,std::vector<std::shared_ptr<State>>>());
            // Go over all Joint History Next
            for (const auto &next_joint_history : next_one_step_uncompressed_occupancy_state->getJointHistories())
            {
                this->all_support[ostate].emplace(next_joint_history,std::vector<std::shared_ptr<State>>());
                // Go over all Hidden State in the next one step uncomppresed occupancy state
                for (const auto next_hidden_state : next_one_step_uncompressed_occupancy_state->getBeliefAt(next_joint_history)->getStates())
                {
                    this->all_support[ostate][next_joint_history].push_back(next_hidden_state);
                }
            }
        }
        return this->createLP(vf,state, t);
    }

    Pair<std::shared_ptr<Action>, double> ActionVFSawtoothLP::selectBestActionRelaxedV2(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state,number t)
    {
        std::shared_ptr<Action> best_action;
        double min_value = std::numeric_limits<double>::max();

        if (vf->getSupport(t + 1).empty())
        {
            // Resolution of the problem when the support of Point Set is empty
            this->all_support = std::unordered_map<std::shared_ptr<State>,std::unordered_map<std::shared_ptr<HistoryInterface>,std::vector<std::shared_ptr<State>>>>();
            auto best_action_AND_value = this->createLP(vf, state, t);

            best_action = best_action_AND_value.first;
            min_value = best_action_AND_value.second;
        }
        else
        {

            // For the Full version of Sawtooth, wo over all the Point Set  
            for (const auto &ostate : vf->getSupport(t+1))
            {
                const auto &next_one_step_uncompressed_occupancy_state = ostate->toOccupancyState()->getOneStepUncompressedOccupancy();

                this->all_support = std::unordered_map<std::shared_ptr<State>,std::unordered_map<std::shared_ptr<HistoryInterface>,std::vector<std::shared_ptr<State>>>>();
                this->all_support.emplace(ostate, std::unordered_map<std::shared_ptr<HistoryInterface>,std::vector<std::shared_ptr<State>>>());

                // Go over all Joint History Next
                for (const auto &next_joint_history : next_one_step_uncompressed_occupancy_state->getJointHistories())
                {
                    this->all_support[ostate].emplace(next_joint_history,std::vector<std::shared_ptr<State>>());
                    // Go over all Hidden State in the next one step uncomppresed occupancy state
                    for (const auto next_hidden_state : next_one_step_uncompressed_occupancy_state->getBeliefAt(next_joint_history)->getStates())
                    {
                        this->all_support[ostate][next_joint_history].push_back(next_hidden_state);
                    }
                }

                auto [action,value] = this->createLP(vf,state, t);

                // std::cout<<"Value Found "<<value<<std::endl;

                if (min_value >value)
                {
                    min_value = value;
                    best_action = action;
                }
            }
        }

        auto resultat = vf->template backup<double>(state,best_action,t);

        auto occupancy_mdp = std::static_pointer_cast<OccupancyMDP>(ActionVFBase::world_);
        auto under_pb = std::dynamic_pointer_cast<MPOMDPInterface>(ActionVFBase::world_->getUnderlyingProblem());

        double reward =0.0;
        for (const auto &joint_history : state->toOccupancyState()->getOneStepUncompressedOccupancy()->getJointHistories())
        {
            // Get the belief corresponding to this history
            auto belief = state->toOccupancyState()->getOneStepUncompressedOccupancy()->getBeliefAt(joint_history);
            // Get the action from decision rule
            auto joint_action = occupancy_mdp->applyDecisionRule(state->toOccupancyState()->getOneStepUncompressedOccupancy(), state->toOccupancyState()->getCompressedJointHistory(joint_history),best_action, t);
            // Update the expected reward
            reward += state->toOccupancyState()->getOneStepUncompressedOccupancy()->getProbability(joint_history) * occupancy_mdp->getUnderlyingBeliefMDP()->getReward(belief, joint_action, t);
        }


        double resultat2 = reward +  vf->evaluate(occupancy_mdp->nextOccupancyState(state, best_action, nullptr, t)->toOccupancyState()->getOneStepUncompressedOccupancy(),t+1).second;

        // std::cout<<"Value LP "<<min_value<<std::endl;
        // std::cout<<"Value Backup "<<resultat2<<std::endl;
        // std::cout<<"Value Backup 2 "<<resultat<<std::endl;

        if(std::abs(min_value - resultat2)>0.01)
        {
            // std::cout<<"Action "<<a.first->str()<<std::endl;

            // std::cout<<"size ? "<<vf->getSupport(t+1).size()<<std::endl;
            // std::cout<<"Erreur "<<std::endl;
            // std::cout<<"LP value : "<<a.second<<", Bakcup Value : "<<resultat2<<", backup "<<resultat<<std::endl;
            // system("cat lb_bellman_op.lp");
            // exit(-1);
        }


        return {best_action, min_value};
    }

    Pair<std::shared_ptr<Action>, double> ActionVFSawtoothLP::selectBestActionRelaxed(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state,number t)
    {
        std::shared_ptr<Action> best_action;
        double min_value = std::numeric_limits<double>::max();


        if (vf->getSupport(t + 1).empty())
        {
            // Resolution of the problem when the support of Point Set is empty
            this->all_support = std::unordered_map<std::shared_ptr<State>,std::unordered_map<std::shared_ptr<HistoryInterface>,std::vector<std::shared_ptr<State>>>>();
            auto best_action_AND_value = this->createLP(vf, state, t);

            best_action = best_action_AND_value.first;
            min_value = best_action_AND_value.second;
        }
        else
        {

            // For the Full version of Sawtooth, wo over all the Point Set  
            for (const auto &ostate : vf->getSupport(t+1))
            {
                const auto &next_one_step_uncompressed_occupancy_state = ostate->toOccupancyState()->getOneStepUncompressedOccupancy();

                double max_value_support = -std::numeric_limits<double>::max();
                std::shared_ptr<Action> best_action_support;

                // Go over all Joint History Next
                for (const auto &next_joint_history : next_one_step_uncompressed_occupancy_state->getJointHistories())
                {
                    // Go over all Hidden State in the next one step uncomppresed occupancy state
                    for (const auto next_hidden_state : next_one_step_uncompressed_occupancy_state->getBeliefAt(next_joint_history)->getStates())
                    {
                        this->all_support = std::unordered_map<std::shared_ptr<State>,std::unordered_map<std::shared_ptr<HistoryInterface>,std::vector<std::shared_ptr<State>>>>();
                        this->all_support.emplace(ostate, std::unordered_map<std::shared_ptr<HistoryInterface>,std::vector<std::shared_ptr<State>>>());
                        this->all_support[ostate].emplace(next_joint_history,std::vector<std::shared_ptr<State>>());
                        this->all_support[ostate][next_joint_history].push_back(next_hidden_state);

                        auto [action,value] = this->createLP(vf,state, t);

                        // We take the best action with the minimum value
                        if (max_value_support < value)
                        {
                            max_value_support = value;
                            best_action_support = action->toAction();
                        }
                    }
                }

                if (min_value >max_value_support)
                {
                    min_value = max_value_support;
                    best_action = best_action_support;
                }
            }
        }
        return {best_action, min_value};
    }


    Pair<std::shared_ptr<Action>, double> ActionVFSawtoothLP::selectBestAction(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t)
    {
        switch (this->type_of_linear_program_)
        {
        case TypeSawtoothLinearProgram::PLAIN_SAWTOOTH_LINER_PROGRAMMING :
            return this->selectBestActionFull(vf,state,t);
            break;
        case TypeSawtoothLinearProgram::RELAXED_SAWTOOTH_LINER_PROGRAMMING :
            return this->selectBestActionRelaxed(vf,state,t);
            break;
        case TypeSawtoothLinearProgram::RELAXED_V2_SAWTOOTH_LINER_PROGRAMMING :
            return this->selectBestActionRelaxedV2(vf,state,t);
            break;

        default:
            break;
        }
    }

    void ActionVFSawtoothLP::createVariables(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t)
    {
        try
        {
            //<! tracking variables
            std::string VarName;

            this->variables.clear();

            //<! 0.b Build variables v_0 = objective variable!
            VarName = this->getVarNameWeight(0);
            var.add(IloNumVar(env, -IloInfinity, vf->getValueAt(state,t), VarName.c_str()));
            this->setNumber(VarName, index++);

            //<! Define variables \omega_k(x',o')

            // Go over all Point Set in t+1
            for (const auto &next_state_AND_All_next_history_AND_All_next_hidden_state : this->all_support)
            {
                const auto &ostate = next_state_AND_All_next_history_AND_All_next_hidden_state.first->toOccupancyState();

                // Go over all Joint History Next
                for (const auto &next_history_AND_All_next_hidden_state : next_state_AND_All_next_history_AND_All_next_hidden_state.second)
                {
                    const auto &next_history = next_history_AND_All_next_hidden_state.first->toJointHistory();

                    // Go over all Hidden State in the next one step uncomppresed occupancy state
                    for (const auto next_hidden_state : next_history_AND_All_next_hidden_state.second)
                    {
                        // <! \omega_k(x',o')
                        VarName = this->getVarNameWeightedStateJointHistory(ostate, next_hidden_state, next_history);
                        var.add(IloBoolVar(env, 0, 1, VarName.c_str()));
                        this->setNumber(VarName, index++);
                    }
                }
            }

            // Create Decentralized Variables
            this->createDecentralizedVariables(vf, state, env, var, index, t);
        }
        catch (const std::exception &exc)
        {
            std::cerr << "SawtoothLPBackup::createVariables(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
    }

    double ActionVFSawtoothLP::getQValueRelaxation(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<Action> &action, number t)
    {
        // \sum_{o} a(u|o) \sum_{x} s(x,o) * Q_MDP(x,u)
        double weight = 0.0;
        auto compressed_occupancy_state = state->toOccupancyState();

        // Relaxation of the problem
        auto relaxation = std::static_pointer_cast<RelaxedValueFunction>(vf->getInitFunction());

        auto belief = compressed_occupancy_state->getBeliefAt(joint_history);
        weight = compressed_occupancy_state->getProbability(joint_history) * relaxation->operator()(std::make_pair(belief, action), t);

        return weight;
    }

    void ActionVFSawtoothLP::createConstraints(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloModel &model, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        assert(vf->getInitFunction() != nullptr);

        //<!  Build sawtooth constraints v - \sum_{u} a(u|o) * Q(k,s,o,u,y,z, diff, t  ) + \omega_k(y,<o,z>)*M <= M,  \forall k, y,<o,z>
        //<!  Build sawtooth constraints  Q(k,s,o,u,y,z, diff, t ) = (v_k - V_k) \frac{\sum_{x} s(x,o) * p(x,u,z,y)}}{s_k(y,<o,z>)},  \forall a(u|o)

        try
        {
            auto compressed_occupancy_state = state->toOccupancyState();

            if(vf->getSupport(t+1).empty())
            {
                this->createInitialConstraints(vf, state,env, con, var,index, t);
            }
            else
            {
                // Go over all points in the point set at t+1
                for (const auto &next_state_AND_All_next_history_AND_All_next_hidden_state : this->all_support)
                {
                    const auto &ostate = next_state_AND_All_next_history_AND_All_next_hidden_state.first;
                    const auto &next_one_step_uncompressed_occupancy_state = ostate->toOccupancyState()->getOneStepUncompressedOccupancy();

                    // Compute the difference i.e. (v_k - V_k)
                    double current_upper_bound = vf->getValueAt(ostate, t+1); //Get Value for the state at t+1
                    double initial_upper_bound = vf->getInitFunction()->operator()(next_one_step_uncompressed_occupancy_state, t + 1); // Get initialisation of the state at t+1
                    
                    double difference = current_upper_bound - initial_upper_bound;

                    // Go over all joint histories in over the support of next_one_step_uncompressed_occupancy_state
                    for (const auto &next_history_AND_All_next_hidden_state : next_state_AND_All_next_history_AND_All_next_hidden_state.second)
                    {
                        const auto &next_history = next_history_AND_All_next_hidden_state.first->toJointHistory();

                        // Go over all Hidden State in the Belief for a precise Joint History
                        for (const auto &next_hidden_state : next_history_AND_All_next_hidden_state.second)
                        {
                            //Determine denominator of the sawtooth ratio
                            double denominator = next_one_step_uncompressed_occupancy_state->getProbability(next_history, next_hidden_state);

                            // Determine the next Joint observation thanks to the next joint history
                            auto next_joint_observation = this->determineNextJointObservation(compressed_occupancy_state, next_history, t);

                            // We search for the joint_history which allow us to obtain the current next_history conditionning to the next joint observation
                            switch (this->current_type_of_resolution_)
                            {
                            case TypeOfResolution::BigM:
                                // this->createSawtoothBigM(compressed_occupancy_state,joint_history,next_hidden_state,next_joint_observation,next_history,next_one_step_uncompressed_occupancy_state,probability,difference,env,con,var,index, t);
                                break;
                            case TypeOfResolution::IloIfThenResolution:
                                this->createSawtoothIloIfThen(vf, compressed_occupancy_state,nullptr, next_hidden_state, next_joint_observation, next_history, ostate, denominator, difference, env, model, var, t);
                                break;
                            }
                        }
                    }
                    this->createOmegaConstraints(ostate,env,con,var,index);
                }
            }
            this->createDecentralizedConstraints(vf, state, env, con, var, index, t);
        }
        catch (const std::exception &exc)
        {
            std::cerr << "ActionVFSawtoothLP::createConstraints(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
    }

    void ActionVFSawtoothLP::createOmegaConstraints(const std::shared_ptr<State> &state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index)
    {
        const auto &ostate = state->toOccupancyState();

        number recover = 0;

        // Build constraint \sum{x',o'} \omega_k(x',o') = 1
        con.add(IloRange(env, 1.0, 1.0));

        // Go over all joint histories in over the support of next_one_step_uncompressed_occupancy_state
        for (const auto &next_history_AND_All_next_hidden_state :  this->all_support[state])
        {
            const auto &next_history = next_history_AND_All_next_hidden_state.first->toJointHistory();

            for (const auto &next_hidden_state : next_history_AND_All_next_hidden_state.second)
            {
                // <! \omega_k(x',o')
                auto VarName = this->getVarNameWeightedStateJointHistory(ostate, next_hidden_state, next_history);
                recover = this->getNumber(VarName);
                con[index].setLinearCoef(var[recover], +1.0);
            }
        }
        index++;
    }
    
    void ActionVFSawtoothLP::createInitialConstraints(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state,  IloEnv &env,IloRangeArray &con, IloNumVarArray &var,number &index, number t)
    {
        auto under_pb = ActionVFBase::world_->getUnderlyingProblem();

        number recover = 0;
        double Qrelaxation;

        con.add(IloRange(env, -IloInfinity, 0));
        con[index].setLinearCoef(var[this->getNumber(this->getVarNameWeight(0))], +1.0);

        // Go over all actions
        for (const auto &action : *under_pb->getActionSpace(t))
        {
            for(const auto& joint_history : state->toOccupancyState()->getJointHistories())
            {
                //<! 1.c.4 get variable a(u|o) and set constant
                recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action->toAction(), joint_history));

                Qrelaxation = this->getQValueRelaxation(vf, state, joint_history, action->toAction(), t);
                con[index].setLinearCoef(var[recover], -Qrelaxation);
            }
        }
        index++;
    }

    double ActionVFSawtoothLP::getSawtoothMinimumRatio(const std::shared_ptr<ValueFunction> &, const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, double denominator, number t)
    {
        double numerator = 0.0;

        try
        {
            // Compute the numerator for the Sawtooth Ratio, i.e. we compute the \sum_{x} s(x,o) * T(x,u,x_,z_)
            // This formulation allow us to tranform the problem at t+1, like the next_one_step_uncompressed_occupancy_state
            // therefore, we can have the sawtooth ration equivalent to 1.

            auto compressed_occupancy_state = state->toOccupancyState();
            auto under_pb = std::dynamic_pointer_cast<MPOMDPInterface>(ActionVFBase::world_->getUnderlyingProblem());

            // Go over all hidden state  in a belief conditionning to a joint history
            for (const auto &hidden_state : compressed_occupancy_state->getBeliefAt(joint_history)->getStates())
            {
                numerator += compressed_occupancy_state->getProbability(joint_history, hidden_state) * under_pb->getDynamics(hidden_state, action, next_hidden_state, next_observation, t);
            }
        }
        catch (const std::exception &exc)
        {
            std::cerr << "ActionVFSawtoothLP::getSawtoothMinimumRatio(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
        return numerator / denominator;
    }

    void ActionVFSawtoothLP::createSawtoothBigM(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> &, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double denominator , double difference , IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        try
        {
            auto under_pb = ActionVFBase::world_->getUnderlyingProblem();

            number recover = 0;
            double Qrelaxation,SawtoothRatio;

            con.add(IloRange(env, -IloInfinity, this->bigM_value_));
            con[index].setLinearCoef(var[this->getNumber(this->getVarNameWeight(0))], +1.0);

            // Go over all actions
            for (const auto &action : *under_pb->getActionSpace(t))
            {
                for(const auto &joint_history : state->toOccupancyState()->getJointHistories())
                {
                    //<! 1.c.4 get variable a(u|o) and set constant
                    recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action->toAction(), joint_history));
                    // con[index].setLinearCoef(var[recover], -this->getQValueRealistic(vf, state, joint_history, action->toAction(), next_hidden_state, next_observation, probability, difference, t));
                    Qrelaxation = this->getQValueRelaxation(vf, state, joint_history, action->toAction(), t);

                    if(joint_history->expand(next_observation) == next_joint_history)
                    {
                        SawtoothRatio = this->getSawtoothMinimumRatio(vf, state, joint_history, action->toAction(), next_hidden_state, next_observation, denominator, t);
                    }else
                    {
                        SawtoothRatio = 0;
                    }

                    //<! 1.c.4 get variable a(u|o) and set constant
                    con[index].setLinearCoef(var[recover],-(Qrelaxation + SawtoothRatio * difference));

                    // <! \omega_k(x',o') * BigM
                    recover = this->getNumber(this->getVarNameWeightedStateJointHistory(next_state, next_hidden_state, next_joint_history));
                    con[index].setLinearCoef(var[recover], this->bigM_value_);
                }
            }
            index++;
        }
        catch (const std::exception &exc)
        {
            // catch anything thrown within try block that derives from std::exception
            std::cerr << "SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedySawtoothBigM(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
    }

    void ActionVFSawtoothLP::createSawtoothIloIfThen(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> &, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double denominator, double difference, IloEnv &env, IloModel &model, IloNumVarArray &var, number t)
    {
        try
        {
            auto under_pb = ActionVFBase::world_->getUnderlyingProblem();
            number recover = 0;

            double Qrelaxation,SawtoothRatio;

            IloExpr expr(env);
            //<! 1.c.1 get variable v and set coefficient of variable v
            expr = var[this->getNumber(this->getVarNameWeight(0))];

            // Go over all actions
            for (const auto &action : *under_pb->getActionSpace(t))
            {
                for(const auto &joint_history : state->toOccupancyState()->getJointHistories())
                {
                    recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action->toAction(), joint_history));

                    Qrelaxation = this->getQValueRelaxation(vf, state, joint_history, action->toAction(), t);

                    if(joint_history->expand(next_observation) == next_joint_history)
                    {
                        Qrelaxation = this->getQValueRelaxation(vf, state, joint_history, action->toAction(), t);
                        SawtoothRatio = this->getSawtoothMinimumRatio(vf, state, joint_history, action->toAction(), next_hidden_state, next_observation, denominator, t);
                    }else
                    {
                        SawtoothRatio = 0;
                    }

                    //<! 1.c.4 get variable a(u|o) and set constant
                    expr -=  (Qrelaxation + SawtoothRatio * difference) * var[recover];
                }
            }

            // <! get variable \omega_k(x',o')
            recover = this->getNumber(this->getVarNameWeightedStateJointHistory(next_state, next_hidden_state, next_joint_history));
            model.add(IloIfThen(env, var[recover] > 0, expr <= 0));
        }
        catch (const std::exception &exc)
        {
            // catch anything thrown within try block that derives from std::exception
            std::cerr << "SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedySawtoothIloIfThen(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
    }

    void ActionVFSawtoothLP::createObjectiveFunction(const std::shared_ptr<ValueFunction> &, const std::shared_ptr<State> &, IloNumVarArray &var, IloObjective &obj, number )
    {
        // <! 1.a get variable v
        auto recover = this->getNumber(this->getVarNameWeight(0));
        obj.setLinearCoef(var[recover], 1);
    }

    void ActionVFSawtoothLP::createDecentralizedVariables(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t)
    {
        return DecentralizedLP::createDecentralizedVariablesOccupancy(vf, state, env, var, index, t);
    }

    void ActionVFSawtoothLP::createDecentralizedConstraints(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        // std::cout << "ActionVFSawtoothLP::createDecentralizedConstraints"<<std::endl;
        return DecentralizedLP::createDecentralizedConstraintsOccupancy(vf, state, env, con, var, index, t);
    }

    std::shared_ptr<Action> ActionVFSawtoothLP::getVariableResult(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const IloCplex &cplex, const IloNumVarArray &var, number t)
    {
        return DecentralizedLP::getVariableResultOccupancy(vf, state, cplex, var, t);
    }

    std::shared_ptr<Joint<std::shared_ptr<Observation>>> ActionVFSawtoothLP::determineNextJointObservation(const std::shared_ptr<State> &, const std::shared_ptr<JointHistoryInterface> &next_joint_history, number )
    {
        return next_joint_history->getLastObservation()->to<Joint<std::shared_ptr<Observation>>>();
    }
}
