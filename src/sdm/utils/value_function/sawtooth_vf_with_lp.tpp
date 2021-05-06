
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
        
        //auto o = std::static_pointer_cast<occupancy_map<typename TState::jhistory_type>>(s);

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
            if(this->representation[t].empty()) this->template setGreedyUpperbound<TState>(occupancy_state, env, con, var, c,t);

            //<! 3.a Build sawtooth constraints v <= sum_{o,u} a(u|o) sum_x s(x,o)  [ \bar{\upsilon}(x,o,u) + \delta_{x,o,u}(x^\prime,o^\prime,u^\prime) \frac{ v_k - \bar{v}_k }{a_k(u^\prime|o^\prime) s_k(x^\prime,o^\prime)} ]  ,\forall k, x^\prime,o^\prime,u^\prime\colon a_k(u^\prime|o^\prime) s_k(x^\prime,o^\prime)>0
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
    void SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedyVariables(const TState& o, std::unordered_map<agent, std::unordered_set<typename TState::jhistory_type::element_type::ihistory_type>>& ihs, IloEnv& env, IloNumVarArray& var, double clb, double cub, number t)
    {
        std::cout<<"\n setGreedyVariables"<<std::endl;

        //<! tracking variable ids
        number index = 0;

        //<! tracking variables
        std::string VarName;

        this->variables.clear();

        //<! 0.b Build variables v_0 = objective variable!
        VarName = this->getVarNameWeight(0);
        var.add(IloNumVar(env, clb, cub, VarName.c_str())); //-IloInfinity, +IloInfinity
        this->setNumber(VarName, index++);

        //<! phi_k, \omega_k(x',o')
        for(auto k : this->representation[t+1])
        {
            //<! phi_k

            VarName = this->getVarNameWeight(k.first); 
            var.add(IloNumVar(env, -IloInfinity, +IloInfinity, VarName.c_str()));
            this->setNumber(VarName, index++);

            auto s_k =  k.first.getOneStepUncompressedOccupancy();
            
            for(const auto& pair_hidden_state_history_proba : *s_k)
            {
                for(auto x : this->getWorld()->getUnderlyingProblem()->getStateSpace()->getAll())
                {
                    if(pair_hidden_state_history_proba.second>0)
                    {
                        // <! \omega_k(x',o')
                        // Est ce qu'on a besoin du x ? car on a déjà cette information dans le k.first
                        VarName = this->getVarNameWeightedStateJointHistory(k.first,x, s_k->getHistory(pair_hidden_state_history_proba.first));
                        var.add(IloBoolVar(env, 0, 1, VarName.c_str()));
                        this->setNumber(VarName, index++);
                    }
                }
            }
        }
        this->template setDecentralizedVariables<TState>(o, ihs, env, var, index);
    }

    template <typename TState, typename TAction, typename TValue>
    void SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedyObjective(IloObjective& obj, IloNumVarArray& var,number t) 
    {
        std::cout<<"\n setGreedyObjective"<<std::endl;

        // <! 1.a get variable v
        auto recover = this->getNumber(this->getVarNameWeight(0));

        //<! 1.b set coefficient of objective function "v"
        obj.setLinearCoef(var[recover], this->getWorld()->getUnderlyingProblem()->getDiscount(t));
    }


    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<>, T>, int>>
    void SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedyUpperbound(const TState& occupancy_state, IloEnv& env, IloRangeArray& con, IloNumVarArray& var, number& c, number t) 
    {

        std::cout<<"\n setGreedyUpperbound"<<std::endl;

        //<! tracking variable ids
        number recover;

        con.add(IloRange(env, -IloInfinity, 0.0));
        //<! 1.c.1 get variable v
        recover = this->getNumber(this->getVarNameWeight(0));
        //<! 1.c.2 set coefficient of variable v
        con[c].setLinearCoef(var[recover], +1.0);

        //<! 1.c.3 set coefficient of variables a(u|o)
        for(const auto &state : occupancy_state)
        {
            auto joint_history =  occupancy_state.getHistory(state.first);
            for(const auto & decision_rule : this->getWorld()->getActionSpaceAt(occupancy_state)->getAll())
            {
                auto u = decision_rule.act(joint_history->getIndividualHistories());
                //<! 1.c.4 get variable a(u|o)
                recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(u,joint_history));

                auto weight = 0.0;
                for(auto x : this->getWorld()->getUnderlyingProblem()->getStateSpace()->getAll())
                {
                    weight +=  this->getQValueAt(x,decision_rule,t)*state.second;
                }
                //<! 1.c.5 set coefficient of variable a(u|o) i.e., \sum_x s(x,o) Q_MDP(x,u)
                con[c].setLinearCoef(var[recover], - weight);
            } //u
        } //jh
        c++;
    }

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<>, T>, int>>
    void SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedySawtooth(const TState& occupancy_state, IloEnv& env, IloRangeArray& con, IloNumVarArray& var, number& c, number t) 
    {

        assert(this->getInitFunction() != nullptr); 

        std::cout<<"\n setGreedySawtooth"<<std::endl;

        number recover = 0;

        for(const auto k : this->representation[t])
        {

            auto s_k = k.first.getOneStepUncompressedOccupancy();
            auto v_k = this->getValueAt(*s_k, t);//k->second.second;  // this->getValueAt(ostate, t) ? 
            auto V_k = this->getInitFunction()->operator()(*s_k, t);//this->getUpperBound( s_k );

            std::cout<<"\n s_k"<<*s_k<<std::endl;
            std::cout<<"\n v_k "<<v_k<<std::endl;
            std::cout<<"\n V_k "<<V_k<<std::endl;
            std::cout<<"\n V_k getUpperBound "<<this->getUpperBound( *s_k,t );

            if(V_k != this->getUpperBound( *s_k,t ))
            {
                // throw sdm::exception::Exception("Diffente value ! this->getInitFunction()->operator()(*s_k, t) : "<<std::to_string(V_k)+", this->getUpperBound( *s_k,t )"+std::to_string(this->getUpperBound( *s_k,t )) );
            }

            //<! setup the constraint w <= 􏰄\sum_{o,u} a(u|o)*V(o,u) + phi_k*(vk−Vk),
            con.add(IloRange(env, -IloInfinity, 0.0));
            //<!  get objective variable
            recover = this->getNumber(this->getVarNameWeight(0));
            //<! setup its coefficient
            con[c].setLinearCoef(var[recover], +1.0);
            //<! get variable phi_k
            recover = this->getNumber(this->getVarNameWeight(k.first));
            //<! setup its coefficient
            con[c].setLinearCoef(var[recover], - (v_k-V_k));

            for(const auto joint_history : occupancy_state.getJointHistories())
            {
                for(const auto & decision_rule : this->getWorld()->getActionSpaceAt(occupancy_state)->getAll())
                {
                    auto u = decision_rule.act(joint_history->getIndividualHistories());
                    //<! get variable a(u|o)
                    recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(u,joint_history));
                    //<! setup its coefficient
                    con[c].setLinearCoef(var[recover], - this->getSawtoothUpperBound(occupancy_state, joint_history, decision_rule,t ));
                }
            }

            //<! increments contraints' id
            c++;

            auto minimum = 1.0;
            //<! setup the constraint \sum_{x',o'} w_k(x',o') = 1
            con.add(IloRange(env, +1.0, +1.0));
            for(const auto& pair_hidden_state_history_proba : *s_k)
            {
                auto joint_history =  s_k->getHistory(pair_hidden_state_history_proba.first);
                auto proba = pair_hidden_state_history_proba.second;
                for(auto x : this->getWorld()->getUnderlyingProblem()->getStateSpace()->getAll())
                {
                    if( proba > 0 )
                    {
                        //<! get variable w_k(x',o')
                        recover = this->getNumber(this->getVarNameWeightedStateJointHistory(k.first,x,joint_history));
                        //<! setup its coefficient
                        con[c].setLinearCoef(var[recover], +1.0);

                        minimum = std::min(minimum, proba);
                    }
                }
            }

            // //<! increments contraints' id
            // c++;

            // // <! setup the constraints \sum_{o,u} 􏰄a(u|o)φk(o,u,x′,o′)+M(1−ωk(x′,o′)) <= phi_k <= \sum_{o,u} 􏰄a(u|o)φk(o,u,x′,o′)+M(1−ωk(x′,o′))
            // for(auto it_=s_k->begin(); it_!=s_k->end(); ++it_)
            // {
            //     for(auto x=0; x<this->getWorld()->getUnderlyingProblem()->getStateSpace()->getNumItems(); ++x) if( s_k->get(x,it_->first) > 0 )
            //     {
            //         //<! setup the sawtooth constraints
            //         this->setGreedySawtoothConstraints(o, s_k, env, con, var, c, k->first, x, it_->first, 10.0/minimum, true);
            //         this->setGreedySawtoothConstraints(o, s_k, env, con, var, c, k->first, x, it_->first, 10.0/minimum, false);
            //     }
            // }
        }
        // throw sdm::exception::NotImplementedException();
    }

    template <typename TState, typename TAction, typename TValue>
    void SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedySawtoothConstraints(const TState& o, const TState& s_k, IloEnv& env, IloRangeArray& con, IloNumVarArray& var, number& c, number k, state x_, typename TState::jhistory_type jh_, double bigM, bool lower_or_equal) 
    {
        std::cout<<"\n setGreedySawtoothConstraints"<<std::endl;

        // //<! setup the constraint
        // if( lower_or_equal )
        // {
        //     con.add(IloRange(env, -IloInfinity, +bigM));
        // } else 
        // {
        //     con.add(IloRange(env, -bigM, +IloInfinity));
        // }

        // //<!  get phi_k variable
        // auto recover = this->getNumber(this->getVarNameWeight(k));
        // //<! setup its coefficient
        // con[c].setLinearCoef(var[recover], +1.0);

        // for(const auto joint_history : occupancy_state.getJointHistories())
        // {
        //     for(action u=0; u<this->getWorld()->getUnderlyingProblem()->getNumActions(); ++u)
        //     {
        //         //<! get variable a(u|o)
        //         recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(u,joint_history));
        //         //<! setup its coefficient
        //         con[c].setLinearCoef(var[recover], - this->getSawtoothMinimumRatio(o, joint_history, u, x_, jh_, s_k));
        //     }
        // }

        // //<! get variable w_k(x',o')
        // recover = this->getNumber(this->getVarNameWeightedStateJointHistory(k,x_,jh_));
        // //<! setup its coefficient
        // if( lower_or_equal ){
        //     con[c].setLinearCoef(var[recover], +bigM);
        // } else{
        //     con[c].setLinearCoef(var[recover], -bigM);
        // }

        // //<! increment constraints' id
        // c++;
        throw sdm::exception::NotImplementedException();
    }

    // Est ce qu'on aura besoin de cette fonction ?
    template <typename TState, typename TAction, typename TValue>
    double SawtoothValueFunctionLP<TState, TAction, TValue>::getUpperBound(const TState& occupancy_state, number t) 
    {
        std::cout<<"\n getUpperBound"<<std::endl;

        double upper_bound = 0.0;

        for(const auto &state : occupancy_state)
        {
            auto joint_history =  occupancy_state.getHistory(state.first);
            for(auto x=0; x<this->getWorld()->getUnderlyingProblem()->getStateSpace()->getNumItems(); ++x)
            {
                upper_bound += this->getValueAt(x,t) * state.second;
            }
        }

        // return upper_bound;
        throw sdm::exception::NotImplementedException();

    }

    // template <typename TState, typename TAction, typename TValue>
    // double SawtoothValueFunctionLP<TState, TAction, TValue>::getSawtoothValueAt(const TState& o_i, double v_i, const TState& s_, double upper_bound) const
    // {
    //     // auto s_0 = std::static_pointer_cast<occupancy_map<typename TState::jhistory_type>>(s_);

    //     // auto ratio_i = 1.0;
    //     // auto s_i = std::static_pointer_cast<occupancy_map<typename TState::jhistory_type>>(o_i);
    //     // auto upper_bound_i = this->getUpperBound(s_i);
    //     // for(auto it=s_i->begin(); it!=s_i->end(); ++it)
    //     // {
    //     //     for(auto x=0; x<this->getWorld()->getUnderlyingProblem()->getStateSpace()->getNumItems(); ++x)
    //     //     {
    //     //         if( s_i->get(x,it->first) > 0 )
    //     //         {
    //     //             ratio_i = std::min(ratio_i, s_0->get(x,it->first) / s_i->get(x,it->first) );
    //     //         }
    //     //     }
    //     // }
    //     // return upper_bound + ratio_i * (v_i-upper_bound_i);
    //     throw sdm::exception::NotImplementedException();
    // }

    // template <typename TState, typename TAction, typename TValue>
    // double SawtoothValueFunctionLP<TState, TAction, TValue>::getSawtoothValueAt(const TState& s, const TState& points) const
    // {
    //     // auto upper_bound = this->getUpperBound(s);
    //     // auto s_0 = std::static_pointer_cast<occupancy_map<typename TState::jhistory_type>>(s);

    //     // auto v_0 = upper_bound;
    //     // for(auto i=points.begin(); i!=points.end(); ++i)
    //     // {
    //     //     v_0 = std::min(v_0, this->getSawtoothValueAt(i->second.first, i->second.second, s_0, upper_bound));
    //     // }
    //     // return v_0;
    //     throw sdm::exception::NotImplementedException();
    // }

    template <typename TState, typename TAction, typename TValue>
    double SawtoothValueFunctionLP<TState, TAction, TValue>::getSawtoothUpperBound(const TState& s, typename TState::jhistory_type jh, TAction u, number t)
    {
        // auto weight = 0.0;
        // for(auto x : this->getWorld()->getUnderlyingProblem()->getStateSpace()->getAll())
        // {
        //     weight += this->getQValueAt(x,u,t); //  * s->get(x, jh);
        // }
        auto weight = this->getQValueAt(s,u,t);//* s->get(x, jh)
        return weight;
    }

    // template <typename TState, typename TAction, typename TValue>
    // double SawtoothValueFunctionLP<TState, TAction, TValue>::getSawtoothMinimumRatio(const TState& s, typename TState::jhistory_type jh, action u, state x_, typename TState::jhistory_type* jh_, const TState& s_k) const
    // {
    //     // auto ratio = 0.0, factor = 0.0;

    //     // for(state x=0; x<this->getWorld()->getUnderlyingProblem()->getStateSpace()->getNumItems(); ++x)
    //     // {
    //     //     factor = 0.0;
    //     //     for(observation z_=0; z_<this->getWorld()->getUnderlyingProblem()->getObsSpace()->getNumItems(); ++z_)
    //     //     {
    //     //         factor +=  jh_ == jh->expand(z_) ? this->getWorld()->getUnderlyingProblem()->getDynamics(x,u,z_,x_) : 0.0;
    //     //     }
    //     //     ratio += factor * s->get(x, jh);
    //     // }

    //     // return ratio / s_k->get(x_,jh_);
    //     throw sdm::exception::NotImplementedException();
    // }

    template <typename TState, typename TAction, typename TValue>
    void SawtoothValueFunctionLP<TState, TAction, TValue>::updateValueAt(const TState &occupancy_state, number t)
    {
        double cub,clb;
        this->greedySawtooth(occupancy_state, clb, cub, t);
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
}
