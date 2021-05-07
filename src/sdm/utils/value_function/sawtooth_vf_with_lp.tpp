
namespace sdm
{

    template <typename TState, typename TAction, typename TValue>
    SawtoothValueFunctionLP<TState, TAction, TValue>::SawtoothValueFunctionLP() {}

    template <typename TState, typename TAction, typename TValue>
    SawtoothValueFunctionLP<TState, TAction, TValue>::SawtoothValueFunctionLP(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, number horizon, std::shared_ptr<Initializer<TState, TAction>> initializer) : SawtoothValueFunction<TState, TAction, TValue>(problem, horizon, initializer), DecentralizedConstraintsLP<TState, TAction, TValue>(problem)
    {
    }

    template <typename TState, typename TAction, typename TValue>
    SawtoothValueFunctionLP<TState, TAction, TValue>::SawtoothValueFunctionLP(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, number horizon, TValue default_value) : SawtoothValueFunctionLP<TState, TAction, TValue>(problem, horizon, std::make_shared<ValueInitializer<TState, TAction>>(default_value))
    {
    }

    template <typename TState, typename TAction, typename TValue>
    TAction SawtoothValueFunctionLP<TState, TAction, TValue>::greedySawtooth(const TState& occupancy_state, double clb, double& cub, number t)
    {
        number c = 0;

        //<! tracking variables
        std::string VarName;

        TAction a;

        std::unordered_map<agent, std::unordered_set<typename TState::jhistory_type::element_type::ihistory_type>> ihs;

        for(number ag=0; ag<this->getWorld()->getUnderlyingProblem()->getNumAgents(); ++ag)
        {
            std::unordered_set<typename TState::jhistory_type::element_type::ihistory_type> empty;
            ihs.emplace(ag, empty);
        }

        IloEnv env;
        try{
            IloModel model(env);

            // Init the model
            IloRangeArray con(env);
            IloNumVarArray var(env);

            IloObjective obj = IloMaximize(env);

            ///////  BEGIN CORE CPLEX Code  ///////

            // 0. Build variables a(u|o), a_i(u_i|o_i), v
            this->setGreedyVariables(occupancy_state, ihs, env, var, clb, cub, t);

            // 1. Build objective function v
            this->setGreedyObjective(obj, var,t);

            //<! 1.c Build mdp constraints v <= \sum_{o,u} a(u|o) \sum_x s(x,o) Q_MDP(x,u)
            if(this->representation[t+1].empty()) this->template setGreedyUpperbound<TState>(occupancy_state, env, con, var, c,t);

            //<! 3.a Build sawtooth constraints v <= \sum_{o,u} a(u|o) \sum_x s(x,o) Q_MDP(x,u) + \sum_{o,u} a(u|o) (v_k - V_k) \frac{\sum_{x,z_} s(x,o)*p(x,u,z_,x_)}}{s_k(x_,o_)} ,\forall k, x_,o_,u_
            else this->template setGreedySawtooth<TState>(occupancy_state,model, env, con, var, c, t);

            // 3. Build decentralized control constraints [  a(u|o) >= \sum_i a_i(u_i|o_i) + 1 - n ] ---- and ---- [ a(u|o) <= a_i(u_i|o_i) ]
            this->template setDecentralizedConstraints<TState>(occupancy_state, ihs, env, con, var, c);

            ///////  END CORE  CPLEX Code ///////
            model.add(obj);
            model.add(con);
            IloCplex cplex(model);
            cplex.setOut(env.getNullStream());
            cplex.setWarning(env.getNullStream());

            // Optimize the problem and obtain solution
            if( !cplex.solve() )
            {
                env.error() << "Failed to optimize MILP" << std::endl;
                cplex.exportModel("bellman_greedy_op.lp");
                system("cat bellman_greedy_op.lp");
                throw(-1);
            }
            else
            {
                cub = cplex.getObjValue();
                a = this->template getDecentralizedVariables<TState>(cplex, var, occupancy_state);
            }
        }catch(IloException& e)
        {
            std::cerr << "Concert exception caught: " << e << std::endl;
            exit(-1);
        }catch (const std::exception &exc)
        {
            // catch anything thrown within try block that derives from std::exception
            std::cerr << "Non-Concert exception caught: " << exc.what() << std::endl;
            exit(-1);
        }

        env.end();

        return a;
    }

    template <typename TState, typename TAction, typename TValue>
    void SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedyVariables(const TState& occupancy_state, std::unordered_map<agent, std::unordered_set<typename TState::jhistory_type::element_type::ihistory_type>>& ihs, IloEnv& env, IloNumVarArray& var, double /*clb*/, double /*cub*/, number t)
    {
        //<! tracking variable ids
        number index = 0;

        //<! tracking variables
        std::string VarName;

        this->variables.clear();

        //<! 0.b Build variables v_0 = objective variable!
        VarName = this->getVarNameWeight(0);
        var.add(IloNumVar(env, -IloInfinity, +IloInfinity, VarName.c_str())); 
        this->setNumber(VarName, index++);

        //<! Define variables \omega_k(x',o')

        // Go over all Point Set in t+1 
        for(auto compressed_occupancy_state_AND_upper_bound : this->representation[t+1])
        {

            auto compressed_occupancy_state = compressed_occupancy_state_AND_upper_bound.first;
            auto one_step_uncompressed_occupancy_state = compressed_occupancy_state.getOneStepUncompressedOccupancy();
           
            // Go over all Joint History Next
            for(const auto& hidden_state_AND_joint_history_AND_probability : *one_step_uncompressed_occupancy_state)
            {
                auto hidden_state = hidden_state_AND_joint_history_AND_probability.first.first;
                auto joint_history = hidden_state_AND_joint_history_AND_probability.first.second;

                // <! \omega_k(x',o')
                VarName = this->getVarNameWeightedStateJointHistory(one_step_uncompressed_occupancy_state, hidden_state, joint_history);
                var.add(IloBoolVar(env, 0, 1, VarName.c_str()));
                this->setNumber(VarName, index++);
           }
        }

        this->template setDecentralizedVariables<TState>(occupancy_state, ihs, env, var, index);
    }

    template <typename TState, typename TAction, typename TValue>
    void SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedyObjective(IloObjective& obj, IloNumVarArray& var,number t) 
    {
        // <! 1.a get variable v
        auto recover = this->getNumber(this->getVarNameWeight(0));

        //<! 1.b set coefficient of objective function "v"
        obj.setLinearCoef(var[recover], this->getWorld()->getUnderlyingProblem()->getDiscount(t));
    }


    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<>, T>, int>>
    void SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedyUpperbound(const TState& occupancy_state, IloEnv& env, IloRangeArray& con, IloNumVarArray& var, number& c, number t) 
    {
        //<! tracking variable ids
        number recover;

        con.add(IloRange(env, -IloInfinity, 0.0));
        //<! 1.c.1 get variable v
        recover = this->getNumber(this->getVarNameWeight(0));
        //<! 1.c.2 set coefficient of variable v
        con[c].setLinearCoef(var[recover], +1.0);

        //<! Build mdp constraints " v - \sum_{o,u} a(u|o) \sum_x s(x,o) Q_MDP(x,u) <=  0 " -- should initializer. 

        // Go over all joint history
        for(const auto &joint_history : occupancy_state.getJointHistories())
        {
            // Go over all action 
            for(const auto & action : this->getWorld()->getUnderlyingProblem()->getActionSpace()->getAll())
            {
                //<! 1.c.4 get variable a(u|o)
                recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action, joint_history));

                //<! 1.c.5 set coefficient of variable a(u|o) i.e., \sum_x s(x,o) Q_MDP(x,u)
                auto weight = this->getQValueRelaxation(occupancy_state, joint_history, action, t);

                con[c].setLinearCoef(var[recover], - weight);
            }
        }
        c++;
    }

    template <typename TState, typename TAction, typename TValue>
    double SawtoothValueFunctionLP<TState, TAction, TValue>::getQValueAt(const TState& compressed_occupancy_state, typename TState::jhistory_type joint_history, typename TAction::output_type action, typename TState::state_type next_hidden_state, typename TState::observation_type next_observation, const TState& next_one_step_uncompressed_occupancy_state, double difference, number t)
    {
        auto upper_bound = this->getQValueRelaxation(compressed_occupancy_state, joint_history, action, t);
        
        upper_bound += difference * this->template getSawtoothMinimumRatio<TState>(*compressed_occupancy_state.getOneStepUncompressedOccupancy(), joint_history, action, next_hidden_state, next_observation, next_one_step_uncompressed_occupancy_state);
        
        return upper_bound;
    }

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<>, T>, int>>
    void SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedySawtooth(const TState& occupancy_state,IloModel& model ,IloEnv& env, IloRangeArray& con, IloNumVarArray& var, number& c, number t) 
    {
        //<!  Build sawtooth constraints v - \sum_{o,u} a(u|o) * Q(k, s,o,u,y,z, diff, t  ) + \omega_k(x',o')*M <= M,  \forall k, y,<o,z>
        //<!  Build sawtooth constraints  Q(k,s,o,u,y,z, diff, t ) = \sum_x s(x,o) Q_MDP(x,u) + (v_k - V_k) \frac{\sum_{x} s(x,o) * p(x,u,z,y)}}{s_k(y,<o,z>)},  \forall a(u|o)

        assert(this->getInitFunction() != nullptr); 
        number recover = 0;

        // Go over all points in the point set at t+1 
        for(const auto compressed_occupancy_state_AND_upper_bound : this->representation[t+1])
        {
            auto upper_bound = compressed_occupancy_state_AND_upper_bound.second;
            auto compressed_occupancy_state = compressed_occupancy_state_AND_upper_bound.first;
            auto initial_upper_bound = this->getInitFunction()->operator()(compressed_occupancy_state, t+1);
            auto next_one_step_uncompressed_occupancy_state = compressed_occupancy_state.getOneStepUncompressedOccupancy();
            auto difference = upper_bound - initial_upper_bound; 

            // Go over all joint histories in over the support of next_one_step_uncompressed_occupancy_state
            for(const auto &pair_hidden_state_AND_joint_history_AND_probability : *next_one_step_uncompressed_occupancy_state)
            {
                //<! Initialize expression
                IloExpr expr(env);

                //<! 1.c.1 get variable v and set coefficient of variable v
                expr = var[this->getNumber(this->getVarNameWeight(0))];

                auto next_hidden_state = pair_hidden_state_AND_joint_history_AND_probability.first.first;
                auto next_joint_history = pair_hidden_state_AND_joint_history_AND_probability.first.second;
                
                auto joint_history = next_joint_history->getParent();
                auto next_observation = next_joint_history->getData();

                // Go over all actions
                for(const auto & action : this->getWorld()->getUnderlyingProblem()->getActionSpace()->getAll())
                {
                    //<! 1.c.4 get variable a(u|o) and set constant 
                    expr -= this->getQValueAt(compressed_occupancy_state, joint_history, action, next_hidden_state, next_observation, *next_one_step_uncompressed_occupancy_state, difference, t) * var[this->getNumber(this->getVarNameJointHistoryDecisionRule(action, joint_history))];
                } 

                // <! get variable \omega_k(x',o')
                recover = this->getNumber(this->getVarNameWeightedStateJointHistory(next_one_step_uncompressed_occupancy_state, next_hidden_state, next_joint_history));
                model.add(IloIfThen(env, var[recover] > 0, expr <= 0 ) );                
            }

            // Build constraint \sum{x',o'} \omega_k(x',o') = 1
            con.add(IloRange(env, 1.0, 1.0));
            for(const auto &pair_hidden_state_AND_joint_history_AND_probability : *next_one_step_uncompressed_occupancy_state)
            {
                auto next_hidden_state = pair_hidden_state_AND_joint_history_AND_probability.first.first;
                auto next_joint_history = pair_hidden_state_AND_joint_history_AND_probability.first.second;
                // <! \omega_k(x',o')
                recover = this->getNumber(this->getVarNameWeightedStateJointHistory(next_one_step_uncompressed_occupancy_state, next_hidden_state, next_joint_history));
                con[c].setLinearCoef(var[recover], +1.0);
            }
            c++;
        }
    }

    template <typename TState, typename TAction, typename TValue>
    double SawtoothValueFunctionLP<TState, TAction, TValue>::getQValueRelaxation(const TState& compressed_occupancy_state,typename TState::jhistory_type joint_history, typename TAction::output_type action, number t) 
    {
        auto weight = 0.0;
        for(auto x : this->getWorld()->getUnderlyingProblem()->getStateSpace()->getAll())
        {
            weight +=  compressed_occupancy_state.at(std::make_pair(x,joint_history)) * std::static_pointer_cast<State2OccupancyValueFunction<typename TState::state_type,TState>>(this->getInitFunction())->getQValueAt(x,this->getWorld()->getUnderlyingProblem()->getActionSpace()->joint2single(action),t);
        }
        return weight;

        //return this->getInitFunction()->operator()(compressed_occupancy_state, t);
    }

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<>, T>, int>>
    double SawtoothValueFunctionLP<TState, TAction, TValue>::getSawtoothMinimumRatio(const TState& one_step_uncompressed_occupancy_state, typename TState::jhistory_type joint_history, typename TAction::output_type action, typename TState::state_type next_hidden_state, typename TState::observation_type next_observation, const TState& next_one_step_uncompressed_occupancy_state)
    {
        auto factor = 0.0;

        for(const auto& hidden_state : one_step_uncompressed_occupancy_state.getStatesAt(joint_history))
        {
            factor += one_step_uncompressed_occupancy_state.at(std::make_pair(hidden_state, joint_history)) * this->getWorld()->getUnderlyingProblem()->getObsDynamics()->getDynamics(hidden_state, this->getWorld()->getUnderlyingProblem()->getActionSpace()->joint2single(action), this->getWorld()->getUnderlyingProblem()->getObsSpace()->joint2single(next_observation), next_hidden_state);
        }

        return factor / next_one_step_uncompressed_occupancy_state.at(std::make_pair(next_hidden_state, joint_history->expand(next_observation)));
    }

    template <typename TState, typename TAction, typename TValue>
    void SawtoothValueFunctionLP<TState, TAction, TValue>::updateValueAt(const TState &occupancy_state, number t)
    {
        double cub,clb;
        auto action = this->greedySawtooth(occupancy_state, clb, cub, t);

        MappedValueFunction<TState,TAction,TValue>::updateValueAt(occupancy_state, t, cub);
    }

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int>>
    void SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedyUpperbound(const TState& occupancy_state, IloEnv& env, IloRangeArray& con, IloNumVarArray& var, number& c, number t) 
    {
        throw sdm::exception::NotImplementedException();
    }

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int>>
    void SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedySawtooth(const TState& occupancy_state, IloModel&, IloEnv& env, IloRangeArray& con, IloNumVarArray& var, number& c, number t) 
    {
    }

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int>>
    double SawtoothValueFunctionLP<TState, TAction, TValue>::getSawtoothMinimumRatio(const TState& occupancy_state, typename TState::jhistory_type jh, typename TAction::output_type u, typename TState::state_type x_, typename TState::jhistory_type jh_, const TState& s_k)
    {}
}
