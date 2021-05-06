
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
            else this->template setGreedySawtooth<TState>(occupancy_state, env, con, var, c, t);

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
    void SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedyVariables(const TState& occupancy_state, std::unordered_map<agent, std::unordered_set<typename TState::jhistory_type::element_type::ihistory_type>>& ihs, IloEnv& env, IloNumVarArray& var, double clb, double cub, number t)
    {
        //<! tracking variable ids
        number index = 0;

        //<! tracking variables
        std::string VarName;

        this->variables.clear();

        //<! 0.b Build variables v_0 = objective variable!
        VarName = this->getVarNameWeight(0);
        var.add(IloNumVar(env, -IloInfinity, +IloInfinity, VarName.c_str())); //-IloInfinity, +IloInfinity
        this->setNumber(VarName, index++);

        //<! \omega_k(x',o')

        // Go over all Point Set in t+1 
        for(auto k : this->representation[t+1])
        {
            auto s_k =  k.first.getOneStepUncompressedOccupancy();

            // Go over all Joint History Next
            for(const auto& joint_history_next : s_k->getJointHistories())
            {
                // Go over all next State
                for(auto x_ : this->getWorld()->getUnderlyingProblem()->getStateSpace()->getAll())
                {
                    if( s_k->at(std::make_pair(x_,joint_history_next))>0)
                    {
                        // <! \omega_k(x',o')
                        VarName = this->getVarNameWeightedStateJointHistory(k.first,x_, joint_history_next);
                        var.add(IloBoolVar(env, 0, 1, VarName.c_str()));
                        this->setNumber(VarName, index++);
                    }
                }
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

        //<! Build mdp constraints v <= \sum_{o,u} a(u|o) \sum_x s(x,o) Q_MDP(x,u)

        // Go over all joint history
        for(const auto &joint_history : occupancy_state.getJointHistories())
        {
            // Go over all action 
            for(const auto & u : this->getWorld()->getUnderlyingProblem()->getActionSpace()->getAll())
            {
                //<! 1.c.4 get variable a(u|o)
                recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(u,joint_history));

                //<! 1.c.5 set coefficient of variable a(u|o) i.e., \sum_x s(x,o) Q_MDP(x,u)
                auto weight = this->getQValueRelaxation(occupancy_state,joint_history,u,t);
                con[c].setLinearCoef(var[recover], - weight);
            }
        }
        c++;
    }

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<>, T>, int>>
    void SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedySawtooth(const TState& occupancy_state, IloEnv& env, IloRangeArray& con, IloNumVarArray& var, number& c, number t) 
    {
        //<!  Build sawtooth constraints v <= \sum_{o,u} a(u|o) \sum_x s(x,o) Q_MDP(x,u) + \sum_{o,u} a(u|o) (v_k - V_k) \frac{\sum_{x,z_} s(x,o)*p(x,u,z_,x_)}}{s_k(x_,o_)} ,\forall k, x_,o_,u_

        assert(this->getInitFunction() != nullptr); 

        number recover = 0;

        // Go over all Point Set in t+1 
        for(const auto k : this->representation[t+1])
        {
            con.add(IloRange(env, -IloInfinity, 0.0));
            //<! 1.c.1 get variable v
            recover = this->getNumber(this->getVarNameWeight(0));
            //<! 1.c.2 set coefficient of variable v
            con[c].setLinearCoef(var[recover], +1.0);

            auto tot=0;

            //<! set v_k, V_k, s_k
            auto s_k = k.first.getOneStepUncompressedOccupancy();
            auto v_k = this->getValueAt(k.first, t+1);
            auto V_k = this->getInitFunction()->operator()(*s_k, t+1);

            // set (v_k - V_k) 
            auto difference_v_k = v_k - V_k; 

            auto bigM = 10;

            // Go over all joint history 
            for(const auto &joint_history : occupancy_state.getJointHistories())
            {
                // Go over all action
                for(const auto & u : this->getWorld()->getUnderlyingProblem()->getActionSpace()->getAll())
                {
                    //<! 1.c.4 get variable a(u|o)
                    recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(u,joint_history));

                    // ***** First part of the equation 

                    //<! set \sum_{o,u} a(u|o) \sum_x s(x,o) Q_MDP(x,u)
                    double first_part = this->getQValueRelaxation(occupancy_state,joint_history,u,t);
                    // con[c].setLinearCoef(var[recover], - weight);

                    // ***** Second part of the equation 

                    //<! 3.a Set \sum_{o,u} a(u|o) \sum_{o,u} a(u|o) (v_k - V_k) \frac{\sum_{x,z_} s(x,o)*p(x,u,z_,x_)}}{s_k(x_,o_)} ,\forall k, x_,o_,u_
                    double second_part=0;
                    for(const auto &joint_history_next : s_k->getJointHistories())
                    {
                        for(auto x_ : this->getWorld()->getUnderlyingProblem()->getStateSpace()->getAll())
                        {
                            if( s_k->at(std::make_pair(x_,joint_history_next))>0)
                            {
                                second_part += difference_v_k * this->template getSawtoothMinimumRatio<TState>(occupancy_state, joint_history, u, x_, joint_history_next, *s_k);
                            }
                        }
                    }

                    // Il faut ajouter faire \sum_{o,u}  a(u|o) * (first_part + second_part)
                    con[c].setLinearCoef(var[recover], -(first_part + second_part));

                    c++;

                } 
            }

            // Third part of the equation

            //<! set (1 - \omega_k(x',o'))*M
            for(const auto& joint_history : s_k->getJointHistories())
            {
                for(auto x : this->getWorld()->getUnderlyingProblem()->getStateSpace()->getAll())
                {
                    if( s_k->at(std::make_pair(x,joint_history))>0)
                    {
                        // <! get variable \omega_k(x',o')
                        recover = this->getNumber(this->getVarNameWeightedStateJointHistory(k.first, x, joint_history));

                        // Il faut faire (1-\omega_k(x',o'))*M
                        //double third_part = (1-var[recover])*bigM;

                        //con[c].setLinearCoef(var[recover], third_part);

                    }
                }
            }

            // Build constraint \sum{x',o'} \omega_k(x',o') = 1

            // Go over all joint history next
            con.add(IloRange(env, 1.0, 1.0));
            for(const auto& joint_history_next : s_k->getJointHistories())
            {
                // Go over all next State
                for(auto x_ : this->getWorld()->getUnderlyingProblem()->getStateSpace()->getAll())
                {
                    if( s_k->at(std::make_pair(x_,joint_history_next))>0)
                    {
                        // <! \omega_k(x',o')
                        recover = this->getNumber(this->getVarNameWeightedStateJointHistory(k.first, x_, joint_history_next));
                        con[c].setLinearCoef(var[recover], +1.0);
                        c++;
                    }
                }
            }
        }
    }

    template <typename TState, typename TAction, typename TValue>
    double SawtoothValueFunctionLP<TState, TAction, TValue>::getQValueRelaxation(const TState& occupancy_state,typename TState::jhistory_type joint_history, typename TAction::output_type u, number t) 
    {
        // auto weight = 0.0;
        // for(auto x : this->getWorld()->getUnderlyingProblem()->getStateSpace()->getAll())
        // {
        //     weight +=  occupancy_state.at(std::make_pair(x,joint_history)); // * this->getQValueAt(x,u,t)
        // }
        auto weight = this->getInitFunction()->operator()(occupancy_state, t);
        return weight;
    }

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<>, T>, int>>
    double SawtoothValueFunctionLP<TState, TAction, TValue>::getSawtoothMinimumRatio(const TState& occupancy_state, typename TState::jhistory_type jh, typename TAction::output_type u, typename TState::state_type x_, typename TState::jhistory_type jh_, const TState& s_k)
    {
        auto ratio = 0.0, factor = 0.0;

        for(const auto x : this->getWorld()->getUnderlyingProblem()->getStateSpace()->getAll())
        {
            factor = 0.0;
            for(const auto z_ : this->getWorld()->getUnderlyingProblem()->getObsSpace()->getAll())
            {
                factor +=  jh_ == jh->expand(z_) ? this->getWorld()->getUnderlyingProblem()->getObsDynamics()->getDynamics(x,this->getWorld()->getUnderlyingProblem()->getActionSpace()->joint2single(u),this->getWorld()->getUnderlyingProblem()->getObsSpace()->joint2single(z_),x_) : 0.0;
            }
            ratio += factor * occupancy_state.at(std::make_pair(x, jh));
        }
        return ratio / s_k.at(std::make_pair(x_,jh_));
    }

    template <typename TState, typename TAction, typename TValue>
    void SawtoothValueFunctionLP<TState, TAction, TValue>::updateValueAt(const TState &occupancy_state, number t)
    {
        double cub,clb;
        auto action = this->greedySawtooth(occupancy_state, clb, cub, t);

        MappedValueFunction<TState,TAction,TValue>::updateValueAt(occupancy_state,t, cub);
    }

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int>>
    void SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedyUpperbound(const TState& occupancy_state, IloEnv& env, IloRangeArray& con, IloNumVarArray& var, number& c, number t) 
    {
        throw sdm::exception::NotImplementedException();
    }

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int>>
    void SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedySawtooth(const TState& occupancy_state, IloEnv& env, IloRangeArray& con, IloNumVarArray& var, number& c, number t) 
    {
    }

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int>>
    double SawtoothValueFunctionLP<TState, TAction, TValue>::getSawtoothMinimumRatio(const TState& occupancy_state, typename TState::jhistory_type jh, typename TAction::output_type u, typename TState::state_type x_, typename TState::jhistory_type jh_, const TState& s_k)
    {}
}
