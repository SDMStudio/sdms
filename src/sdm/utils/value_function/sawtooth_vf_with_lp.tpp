
namespace sdm
{

    template <typename TState, typename TAction, typename TValue>
    SawtoothValueFunctionLP<TState, TAction, TValue>::SawtoothValueFunctionLP()
    {
        this->setTStateType(ONE_STEP_UNCOMPRESSED);
        this->setSawtoothType(PLAIN_SAWTOOTH_LINER_PROGRAMMING);
    }

    template <typename TState, typename TAction, typename TValue>
    SawtoothValueFunctionLP<TState, TAction, TValue>::SawtoothValueFunctionLP(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, number horizon, std::shared_ptr<Initializer<TState, TAction>> initializer, TypeOfResolution current_type_of_resolution, number bigM_value, TypeSawtoothLinearProgram type_sawtooth_resolution)
        : DecentralizedConstraintsLP<TState, TAction, TValue>(problem),
          SawtoothValueFunction<TState, TAction, TValue>(problem, horizon, initializer),
          current_type_of_resolution_(current_type_of_resolution),
          bigM_value_(bigM_value)
    {
        this->setTStateType(ONE_STEP_UNCOMPRESSED);
        this->setSawtoothType(type_sawtooth_resolution);
    }

    template <typename TState, typename TAction, typename TValue>
    SawtoothValueFunctionLP<TState, TAction, TValue>::SawtoothValueFunctionLP(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, number horizon, TValue default_value, TypeOfResolution current_type_of_resolution, number bigM_value, TypeSawtoothLinearProgram type_sawtooth_resolution)
        : SawtoothValueFunctionLP<TState, TAction, TValue>(problem, horizon, std::make_shared<ValueInitializer<TState, TAction>>(default_value), current_type_of_resolution, bigM_value, type_sawtooth_resolution)
    {
    }

    template <typename TState, typename TAction, typename TValue>
    TState_t SawtoothValueFunctionLP<TState, TAction, TValue>::getTStateType()
    {
        return this->ctype;
    }

    template <typename TState, typename TAction, typename TValue>
    void SawtoothValueFunctionLP<TState, TAction, TValue>::setTStateType(const TState_t &ctype)
    {
        this->ctype = ctype;
    }

    template <typename TState, typename TAction, typename TValue>
    TypeSawtoothLinearProgram SawtoothValueFunctionLP<TState, TAction, TValue>::getSawtoothType()
    {
        return this->csawtooth_lp_;
    }

    template <typename TState, typename TAction, typename TValue>
    void SawtoothValueFunctionLP<TState, TAction, TValue>::setSawtoothType(const TypeSawtoothLinearProgram &csawtooth_lp)
    {
        this->csawtooth_lp_ = csawtooth_lp;
    }

    template <typename TState, typename TAction, typename TValue>
    TValue SawtoothValueFunctionLP<TState, TAction, TValue>::getBackup(const TState &compressed_occupancy_state, number t)
    {
        double cub = 0;
        this->greedySawtooth(compressed_occupancy_state, cub, t);
        return cub;
    }

    template <typename TState, typename TAction, typename TValue>
    TAction SawtoothValueFunctionLP<TState, TAction, TValue>::getBestAction(const TState &compressed_occupancy_state, number t)
    {
        double cub = 0;
        return this->greedySawtooth(compressed_occupancy_state, cub, t);
    }

    template <typename TState, typename TAction, typename TValue>
    TAction SawtoothValueFunctionLP<TState, TAction, TValue>::greedySawtooth(const TState &occupancy_state, double &cub, number t)
    {
        switch (this->getSawtoothType())
        {
        case TypeSawtoothLinearProgram::PLAIN_SAWTOOTH_LINER_PROGRAMMING:
            return this->greedyFullSawtooth(occupancy_state, cub, t);
        case TypeSawtoothLinearProgram::RELAXED_SAWTOOTH_LINER_PROGRAMMING:
            return this->greedyRelaxedSawtooth(occupancy_state, cub, t);
        default:
            return this->greedyFullSawtooth(occupancy_state, cub, t);
        }
    }

    template <typename TState, typename TAction, typename TValue>
    TValue SawtoothValueFunctionLP<TState, TAction, TValue>::getValueAt(const TState &state, number t)
    {
        TState state_;
        switch (this->ctype)
        {
        case TState_t::FULLY_UNCOMPRESSED:
            state_ = *state.getFullyUncompressedOccupancy();
            break;
        case TState_t::ONE_STEP_UNCOMPRESSED:
            state_ = *state.getOneStepUncompressedOccupancy();
            break;
        default:
            state_ = state;
            break;
        }
        return SawtoothValueFunction<TState, TAction, TValue>::getValueAt(state_, t);
    }

    template <typename TState, typename TAction, typename TValue>
    void SawtoothValueFunctionLP<TState, TAction, TValue>::updateValueAt(const TState &state, number t)
    {
        TState state_;
        switch (this->ctype)
        {
        case TState_t::FULLY_UNCOMPRESSED:
            state_ = *state.getFullyUncompressedOccupancy();
            break;
        case TState_t::ONE_STEP_UNCOMPRESSED:
            state_ = *state.getOneStepUncompressedOccupancy();
            break;
        default:
            state_ = state;
            break;
        }
        SawtoothValueFunction<TState, TAction, TValue>::updateValueAt(state_, t);
    }

    template <typename TState, typename TAction, typename TValue>
    TAction SawtoothValueFunctionLP<TState, TAction, TValue>::greedyRelaxedSawtooth(const TState &occupancy_state, double &cub, number t)
    {
        TAction greedy, action;
        double value;
        cub = std::numeric_limits<double>::max();

        //// DEBUG //////
        // double vub;
        // auto gaction = greedyFullSawtooth(occupancy_state, vub, t);
        /////////////////

        if (this->representation[t + 1].empty())
        {
            this->tmp_representation = {};
            greedy = this->greedyActionSelectionBySawtooth(occupancy_state, cub, t);
        }

        else
        {
            for (const auto &element : this->representation[t + 1])
            {
                this->tmp_representation = {element};

                action = this->greedyActionSelectionBySawtooth(occupancy_state, value, t);

                value = this->getQValueAt(occupancy_state, action, t);

                //if( this->representation[t+1].size() > 1 ) std::cout << "\thorizon=" << t << "\tvalue=" << value << "\t cub=" << cub << "\t vub=" << vub << std::endl;

                if (cub > value)
                {
                    cub = value;
                    greedy = action;
                }
            }
        }

        return greedy;
    }

    template <typename TState, typename TAction, typename TValue>
    TAction SawtoothValueFunctionLP<TState, TAction, TValue>::greedyFullSawtooth(const TState &occupancy_state, double &cub, number t)
    {
        this->tmp_representation = this->representation[t + 1];
        return this->greedyActionSelectionBySawtooth(occupancy_state, cub, t);
    }

    template <typename TState, typename TAction, typename TValue>
    TAction SawtoothValueFunctionLP<TState, TAction, TValue>::greedyActionSelectionBySawtooth(const TState &occupancy_state, double &cub, number t)
    {
        number index = 0;

        //<! tracking variables
        std::string VarName;

        TAction a;

        IloEnv env;
        try
        {
            IloModel model(env);

            // Init the model
            IloRangeArray con(env);
            IloNumVarArray var(env);

            IloObjective obj = IloMaximize(env);

            ///////  BEGIN CORE CPLEX Code  ///////

            // 0. Build variables a(u|o), a_i(u_i|o_i), v
            this->setGreedyVariables(occupancy_state, env, var, cub, t);

            // 1. Build objective function \sum_{o,u} a(u|o) \sum_x s(x,o) Q_MDP(x,u) - discount * v
            this->template setGreedyObjective<TState>(occupancy_state, obj, var, t);

            //<! 3.a Build sawtooth constraints v <= (V_k - v_k) * \frac{\sum_{o,u} a(u|o)\sum_{x,z_} s(x,o)*p(x,u,z_,x_)}}{s_k(x_,o_)} ,\forall k, x_,o_
            this->template setGreedySawtooth<TState>(occupancy_state, model, env, con, var, index, t);

            // 3. Build decentralized control constraints [  a(u|o) >= \sum_i a_i(u_i|o_i) + 1 - n ] ---- and ---- [ a(u|o) <= a_i(u_i|o_i) ]
            this->template setDecentralizedConstraints<TState>(occupancy_state, env, con, var, index, t);

            ///////  END CORE  CPLEX Code ///////
            model.add(obj);
            model.add(con);
            IloCplex cplex(model);
            cplex.setOut(env.getNullStream());
            cplex.setWarning(env.getNullStream());

            // Optimize the problem and obtain solution
            cplex.exportModel("bellman_greedy_op.lp");
            if (!cplex.solve())
            {
                env.error() << "Failed to optimize MILP" << std::endl;
                system("cat bellman_greedy_op.lp");
                throw(-1);
            }
            else
            {
                cub = cplex.getObjValue();
                a = this->template getDecentralizedVariables<TState>(cplex, var, occupancy_state, t);
            }
        }
        catch (IloException &e)
        {
            std::cerr << "Concert exception caught: " << e << std::endl;
            exit(-1);
        }
        catch (const std::exception &exc)
        {
            // catch anything thrown within try block that derives from std::exception
            std::cerr << "Non-Concert exception caught: " << exc.what() << std::endl;
            exit(-1);
        }

        env.end();

        return a;
    }

    template <typename TState, typename TAction, typename TValue>
    void SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedyVariables(const TState &occupancy_state, IloEnv &env, IloNumVarArray &var, double /*cub*/, number t)
    {
        try
        {
            //<! tracking variable ids
            number index = 0;

            //<! tracking variables
            std::string VarName;

            this->variables.clear();

            //<! 0.b Build variables v_0 = objective variable!
            VarName = this->getVarNameWeight(0);
            var.add(IloNumVar(env, -IloInfinity, 0.0, VarName.c_str()));
            this->setNumber(VarName, index++);

            //<! Define variables \omega_k(x',o')

            // Go over all Point Set in t+1
            for (const auto &next_one_step_uncompressed_occupancy_state_AND_upper_bound : this->tmp_representation)
            {
                const auto &next_one_step_uncompressed_occupancy_state = next_one_step_uncompressed_occupancy_state_AND_upper_bound.first;

                // Go over all Joint History Next
                for (const auto &hidden_state_AND_joint_history_AND_probability : next_one_step_uncompressed_occupancy_state)
                {
                    auto hidden_state_AND_joint_history = hidden_state_AND_joint_history_AND_probability.first;

                    auto hidden_state = next_one_step_uncompressed_occupancy_state.getState(hidden_state_AND_joint_history);
                    auto joint_history = next_one_step_uncompressed_occupancy_state.getHistory(hidden_state_AND_joint_history);

                    // <! \omega_k(x',o')
                    VarName = this->getVarNameWeightedStateJointHistory(next_one_step_uncompressed_occupancy_state, hidden_state, joint_history);
                    var.add(IloBoolVar(env, 0, 1, VarName.c_str()));
                    this->setNumber(VarName, index++);
                }
            }
            this->template setDecentralizedVariables<TState>(occupancy_state, env, var, index, t);
        }
        catch (const std::exception &exc)
        {
            // catch anything thrown within try block that derives from std::exception
            std::cerr << "SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedyVariables(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
    }

    template <typename TState, typename TAction, typename TValue>
    double SawtoothValueFunctionLP<TState, TAction, TValue>::getQValueRealistic(const TState &compressed_occupancy_state, typename TState::jhistory_type joint_history, typename TAction::output_type action, typename TState::state_type next_hidden_state, typename TState::observation_type next_observation, double denominator, double difference)
    {
        return difference * this->template getSawtoothMinimumRatio<TState>(compressed_occupancy_state, joint_history, action, next_hidden_state, next_observation, denominator);
    }

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_any<T, OccupancyState<>, OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>>::value, int>>
    void SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedyObjective(const TState &compressed_occupancy_state, IloObjective &obj, IloNumVarArray &var, number t)
    {
        try
        {
            // <! 1.a get variable v
            auto recover = this->getNumber(this->getVarNameWeight(0));

            //<! 1.b set coefficient of objective function "\sum_{o,u} a(u|o) \sum_x s(x,o) Q_MDP(x,u) - discount * v0"
            obj.setLinearCoef(var[recover], this->getWorld()->getUnderlyingProblem()->getDiscount(t));

            // Go over all action
            for (const auto &action : this->getWorld()->getUnderlyingProblem()->getActionSpace()->getAll())
            {
                // Go over all joint history
                for (const auto &joint_history : compressed_occupancy_state.getJointHistories())
                {
                    //<! 1.c.4 get variable a(u|o)
                    recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action, joint_history));

                    //<! 1.c.5 set coefficient of variable a(u|o) i.e., \sum_x s(x,o) Q_MDP(x,u)
                    obj.setLinearCoef(var[recover], this->template getQValueRelaxation<TState>(compressed_occupancy_state, joint_history, action, t));
                }
            }
        }
        catch (const std::exception &exc)
        {
            // catch anything thrown within try block that derives from std::exception
            std::cerr << "SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedyObjective(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
    }

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<>, T>, int>>
    double SawtoothValueFunctionLP<TState, TAction, TValue>::getQValueRelaxation(const TState &compressed_occupancy_state, typename TState::jhistory_type joint_history, typename TAction::output_type action, number t)
    {
        // \sum_{o} a(u|o) \sum_{x} s(x,o) * Q_MDP(x,u)

        auto weight = 0.0;
        try
        {
            auto relaxation = std::static_pointer_cast<BaseRelaxedValueFunction<TState>>(this->getInitFunction());
            auto index_action = this->getWorld()->getUnderlyingProblem()->getActionSpace()->joint2single(action);

            if (relaxation->isPomdpAvailable())
            {
                const auto &list_states = compressed_occupancy_state.getStatesAt(joint_history);
                BeliefState<> belief;
                for (const auto &x : list_states)
                {
                    belief.setProbabilityAt(x, compressed_occupancy_state.at(std::make_pair(x, joint_history)));
                }
                weight = std::static_pointer_cast<RelaxedValueFunction<BeliefState<>, TState>>(this->getInitFunction())->operator()(std::make_pair(belief, index_action), t);
            }
            else
            {
                for (const auto &x : compressed_occupancy_state.getStatesAt(joint_history))
                {
                    // \sum_{x} s(x,o) * Q_MDP(x,u)
                    weight += compressed_occupancy_state.at(std::make_pair(x, joint_history)) * std::static_pointer_cast<RelaxedValueFunction<typename TState::state_type, TState>>(this->getInitFunction())->operator()(std::make_pair(x, index_action), t);
                }
            }
        }
        catch (const std::exception &exc)
        {
            // catch anything thrown within try block that derives from std::exception
            std::cerr << "SawtoothValueFunctionLP<TState, TAction, TValue>::getQValueRelaxation(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
        return weight;
    }

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_any<T, OccupancyState<>, OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>>::value, int>>
    void SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedySawtooth(const TState &compressed_occupancy_state, IloModel &model, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        number recover = 0;

        // Build constraints \sum_x s(x,o) Q_MDP(x,u) + discount * v0 <= \bar{v}(st)
        TState one_step_uncompressed_occupancy_state = *compressed_occupancy_state.getOneStepUncompressedOccupancy();

        //By default, the upper bound of the compressed is v_relaxation(st)
        double upper_bound_compressed = this->getInitFunction()->operator()(compressed_occupancy_state, t);

        //Try to find a better upper for \bar{v}(st)
        for (const auto &one_step_uncompressed_occupancy_state_AND_upper_bound : this->representation[t])
        {
            //Successully find a better upper bound
            if (one_step_uncompressed_occupancy_state_AND_upper_bound.first == one_step_uncompressed_occupancy_state)
            {
                upper_bound_compressed = one_step_uncompressed_occupancy_state_AND_upper_bound.second;
            }
        }
        // Add range contraints
        con.add(IloRange(env, -IloInfinity, upper_bound_compressed));

        recover = this->getNumber(this->getVarNameWeight(0));
        //<! 1.b set coefficient of objective function "\sum_{o,u} a(u|o) \sum_x s(x,o) Q_MDP(x,u) + discount * v0"
        con[index].setLinearCoef(var[recover], this->getWorld()->getUnderlyingProblem()->getDiscount(t));

        // Go over all action
        for (const auto &action : this->getWorld()->getUnderlyingProblem()->getActionSpace()->getAll())
        {
            // Go over all joint history
            for (const auto &joint_history : compressed_occupancy_state.getJointHistories())
            {
                //<! 1.c.4 get variable a(u|o)
                recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action, joint_history));

                //<! 1.c.5 set coefficient of variable a(u|o) i.e., \sum_x s(x,o) Q_MDP(x,u)
                con[index].setLinearCoef(var[recover], this->template getQValueRelaxation<TState>(compressed_occupancy_state, joint_history, action, t));
            }
        }
        index++;

        //<!  Build sawtooth constraints v - \sum_{u} a(u|o) * Q(k,s,o,u,y,z, diff, t  ) + \omega_k(y,<o,z>)*M <= M,  \forall k, y,<o,z>
        //<!  Build sawtooth constraints  Q(k,s,o,u,y,z, diff, t ) = (v_k - V_k) \frac{\sum_{x} s(x,o) * p(x,u,z,y)}}{s_k(y,<o,z>)},  \forall a(u|o)

        assert(this->getInitFunction() != nullptr);

        try
        {
            // Go over all points in the point set at t+1
            for (const auto &next_one_step_uncompressed_occupancy_state_AND_upper_bound : this->tmp_representation)
            {
                const auto &next_one_step_uncompressed_occupancy_state = next_one_step_uncompressed_occupancy_state_AND_upper_bound.first;
                auto current_upper_bound = next_one_step_uncompressed_occupancy_state_AND_upper_bound.second;

                // Compute the difference i.e. (v_k - V_k)
                auto initial_upper_bound = this->getInitFunction()->operator()(next_one_step_uncompressed_occupancy_state, t + 1);
                auto difference = current_upper_bound - initial_upper_bound;

                // Go over all joint histories in over the support of next_one_step_uncompressed_occupancy_state
                for (const auto &pair_hidden_state_AND_joint_history_AND_probability : next_one_step_uncompressed_occupancy_state)
                {
                    // Takes all the necessary variables for computation
                    auto probability = pair_hidden_state_AND_joint_history_AND_probability.second;
                    auto hidden_state_AND_joint_history = pair_hidden_state_AND_joint_history_AND_probability.first;

                    auto next_hidden_state = next_one_step_uncompressed_occupancy_state.getState(hidden_state_AND_joint_history);
                    auto next_joint_history = next_one_step_uncompressed_occupancy_state.getHistory(hidden_state_AND_joint_history);

                    auto joint_history = next_joint_history->getParent();
                    auto next_observation = next_joint_history->getData();

                    // Build the sawtooth constraint depending on the type of resolution choiced
                    switch (this->current_type_of_resolution_)
                    {
                    case TypeOfResolution::BigM:
                        this->template setGreedySawtoothBigM<TState>(compressed_occupancy_state, joint_history, next_hidden_state, next_observation, next_joint_history, next_one_step_uncompressed_occupancy_state, probability, difference, env, con, var, index, t);
                        break;
                    case TypeOfResolution::IloIfThenResolution:
                        this->template setGreedySawtoothIloIfThen<TState>(compressed_occupancy_state, joint_history, next_hidden_state, next_observation, next_joint_history, next_one_step_uncompressed_occupancy_state, probability, difference, env, model, var, t);
                        break;
                    }
                }

                // Build constraint \sum{x',o'} \omega_k(x',o') = 1
                con.add(IloRange(env, 1.0, 1.0));
                for (const auto &pair_hidden_state_AND_joint_history_AND_probability : next_one_step_uncompressed_occupancy_state)
                {
                    auto hidden_state_AND_joint_history = pair_hidden_state_AND_joint_history_AND_probability.first;

                    auto next_hidden_state = next_one_step_uncompressed_occupancy_state.getState(hidden_state_AND_joint_history);
                    auto next_joint_history = next_one_step_uncompressed_occupancy_state.getHistory(hidden_state_AND_joint_history);

                    // <! \omega_k(x',o')
                    auto VarName = this->getVarNameWeightedStateJointHistory(next_one_step_uncompressed_occupancy_state, next_hidden_state, next_joint_history);

                    recover = this->getNumber(VarName);
                    con[index].setLinearCoef(var[recover], +1.0);
                }
                index++;
            }
        }
        catch (const std::exception &exc)
        {
            // catch anything thrown within try block that derives from std::exception
            std::cerr << "SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedySawtooth(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
    }

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_any<T, OccupancyState<>, OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>>::value, int>>
    void SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedySawtoothBigM(const TState &compressed_occupancy_state, typename TState::jhistory_type &joint_history, typename TState::state_type &next_hidden_state, typename TState::observation_type &next_observation, typename TState::jhistory_type &next_joint_history, const TState &next_one_step_uncompressed_occupancy_state, double probability, double difference, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number)
    {
        try
        {
            con.add(IloRange(env, -IloInfinity, this->bigM_value_));
            con[index].setLinearCoef(var[this->getNumber(this->getVarNameWeight(0))], +1.0);

            if (compressed_occupancy_state.getJointHistories().find(joint_history) != compressed_occupancy_state.getJointHistories().end())
            {
                // Go over all actions
                for (const auto &action : this->getWorld()->getUnderlyingProblem()->getActionSpace()->getAll())
                {
                    //<! 1.c.4 get variable a(u|o) and set constant
                    con[index].setLinearCoef(var[this->getNumber(this->getVarNameJointHistoryDecisionRule(action, joint_history))], -this->getQValueRealistic(compressed_occupancy_state, joint_history, action, next_hidden_state, next_observation, probability, difference));
                }
            }

            // <! \omega_k(x',o') * BigM
            auto VarName = this->getVarNameWeightedStateJointHistory(next_one_step_uncompressed_occupancy_state, next_hidden_state, next_joint_history);
            con[index].setLinearCoef(var[this->getNumber(VarName)], this->bigM_value_);

            index++;
        }
        catch (const std::exception &exc)
        {
            // catch anything thrown within try block that derives from std::exception
            std::cerr << "SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedySawtoothBigM(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
    }

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_any<T, OccupancyState<>, OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>>::value, int>>
    void SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedySawtoothIloIfThen(const TState &compressed_occupancy_state, typename TState::jhistory_type &joint_history, typename TState::state_type &next_hidden_state, typename TState::observation_type &next_observation, typename TState::jhistory_type &next_joint_history, const TState &next_one_step_uncompressed_occupancy_state, double probability, double difference, IloEnv &env, IloModel &model, IloNumVarArray &var, number)
    {
        number recover = 0;

        try
        {
            IloExpr expr(env);
            //<! 1.c.1 get variable v and set coefficient of variable v
            expr = var[this->getNumber(this->getVarNameWeight(0))];

            if (compressed_occupancy_state.getJointHistories().find(joint_history) != compressed_occupancy_state.getJointHistories().end())
            {
                // Go over all actions
                for (const auto &action : this->getWorld()->getUnderlyingProblem()->getActionSpace()->getAll())
                {
                    //<! 1.c.4 get variable a(u|o) and set constant
                    expr -= this->getQValueRealistic(compressed_occupancy_state, joint_history, action, next_hidden_state, next_observation, probability, difference) * var[this->getNumber(this->getVarNameJointHistoryDecisionRule(action, joint_history))];
                }
            }

            // <! get variable \omega_k(x',o')
            recover = this->getNumber(this->getVarNameWeightedStateJointHistory(next_one_step_uncompressed_occupancy_state, next_hidden_state, next_joint_history));
            model.add(IloIfThen(env, var[recover] > 0, expr <= 0));
        }
        catch (const std::exception &exc)
        {
            // catch anything thrown within try block that derives from std::exception
            std::cerr << "SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedySawtoothIloIfThen(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
    }

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<>, T>, int>>
    double SawtoothValueFunctionLP<TState, TAction, TValue>::getSawtoothMinimumRatio(const TState &compressed_occupancy_state, typename TState::jhistory_type joint_history, typename TAction::output_type action, typename TState::state_type next_hidden_state, typename TState::observation_type next_observation, double denominator)
    {
        auto factor = 0.0;

        try
        {
            // Go over all state conditionning to a joint history
            for (const auto &hidden_state : compressed_occupancy_state.getStatesAt(joint_history))
            {
                // \sum_{x} s(x,o) * p_{x,u,z',x'}
                factor += compressed_occupancy_state.at(std::make_pair(hidden_state, joint_history)) * this->getWorld()->getUnderlyingProblem()->getObsDynamics()->getDynamics(hidden_state, this->getWorld()->getUnderlyingProblem()->getActionSpace()->joint2single(action), this->getWorld()->getUnderlyingProblem()->getObsSpace()->joint2single(next_observation), next_hidden_state);
            }
        }
        catch (const std::exception &exc)
        {
            // catch anything thrown within try block that derives from std::exception
            std::cerr << "SawtoothValueFunctionLP<TState, TAction, TValue>::getSawtoothMinimumRatio(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }

        return factor / denominator;
    }

    // --------------------------------------------------------------------------
    // -------------  SerializedOccupancyState<TState, JointHistory>  ----------------
    // --------------------------------------------------------------------------

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int>>
    void SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedySawtooth(const TState &compressed_serial_occupancy_state, IloModel &model, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        number recover = 0;

        // Go over all points in the point set at t+1
        for (const auto &next_one_step_uncompressed_serial_occupancy_state_AND_upper_bound : this->tmp_representation)
        {

            auto &next_one_step_uncompressed_serial_occupancy_state = next_one_step_uncompressed_serial_occupancy_state_AND_upper_bound.first;

            auto initial_upper_bound = this->getInitFunction()->operator()(next_one_step_uncompressed_serial_occupancy_state, t + 1);
            auto difference = next_one_step_uncompressed_serial_occupancy_state_AND_upper_bound.second - initial_upper_bound;

            auto next_agent_id = next_one_step_uncompressed_serial_occupancy_state.getCurrentAgentId();

            // Go over all individual histories in over the support of next_one_step_uncompressed_serial_occupancy_state
            for (const auto &pair_hidden_serial_state_AND_joint_history_AND_probability : next_one_step_uncompressed_serial_occupancy_state)
            {
                // Takes all the necessary variables for computation
                auto probability = pair_hidden_serial_state_AND_joint_history_AND_probability.second;
                auto pair_hidden_serial_state_AND_joint_history = pair_hidden_serial_state_AND_joint_history_AND_probability.first;

                auto next_hidden_serial_state = next_one_step_uncompressed_serial_occupancy_state.getState(pair_hidden_serial_state_AND_joint_history);
                auto next_joint_history = next_one_step_uncompressed_serial_occupancy_state.getHistory(pair_hidden_serial_state_AND_joint_history);

                // Gets the current joint history conditional upon the current agent and next joint history
                auto joint_history = next_agent_id == 0 ? next_joint_history->getParent() : next_joint_history;
                // Gets the current joint observation conditional upon the current agent and next joint history
                auto next_observation = next_agent_id == 0 ? next_joint_history->getData() : next_joint_history->getDefaultObs();

                // Build the sawtooth constraint depending on the type of resolution choiced
                switch (this->current_type_of_resolution_)
                {
                case TypeOfResolution::BigM:
                    this->template setGreedySawtoothBigM<TState>(compressed_serial_occupancy_state, joint_history, next_hidden_serial_state, next_observation, next_joint_history, next_one_step_uncompressed_serial_occupancy_state, probability, difference, env, con, var, index, t);
                    break;
                case TypeOfResolution::IloIfThenResolution:
                    this->template setGreedySawtoothIloIfThen<TState>(compressed_serial_occupancy_state, joint_history, next_hidden_serial_state, next_observation, next_joint_history, next_one_step_uncompressed_serial_occupancy_state, probability, difference, env, model, var, t);
                    break;
                }
            }

            // Build constraint \sum{x',o'} \omega_k(x',o') = 1
            con.add(IloRange(env, 1.0, 1.0));
            for (const auto &pair_hidden_state_AND_joint_history_AND_probability : next_one_step_uncompressed_serial_occupancy_state)
            {
                auto pair_hidden_serial_state_AND_joint_history = pair_hidden_state_AND_joint_history_AND_probability.first;
                auto next_hidden_serial_state = next_one_step_uncompressed_serial_occupancy_state.getState(pair_hidden_serial_state_AND_joint_history);
                auto next_joint_history = next_one_step_uncompressed_serial_occupancy_state.getHistory(pair_hidden_serial_state_AND_joint_history);

                // <! \omega_k(x',o')
                recover = this->getNumber(this->getVarNameWeightedStateJointHistory(next_one_step_uncompressed_serial_occupancy_state, next_hidden_serial_state, next_joint_history));
                con[index].setLinearCoef(var[recover], +1.0);
            }
            index++;
        }
    }

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int>>
    double SawtoothValueFunctionLP<TState, TAction, TValue>::getSawtoothMinimumRatio(const TState &compressed_serial_occupancy_state, typename TState::jhistory_type joint_history, typename TAction::output_type action, typename TState::state_type next_hidden_serial_state, typename TState::observation_type next_observation, double denominator)
    {
        double factor = 0.0;

        // Go over all hidden serial state  conditional to a joint_history
        for (const auto &hidden_serial_state : compressed_serial_occupancy_state.getStatesAt(joint_history))
        {
            // \sum_{x} s(x,o) * p_{x,u,z',x'}
            factor += compressed_serial_occupancy_state.at(std::make_pair(hidden_serial_state, joint_history)) * this->getWorld()->getUnderlyingProblem()->getDynamics(hidden_serial_state, action, next_observation, next_hidden_serial_state);
        }

        return factor / denominator;
    }

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int>>
    double SawtoothValueFunctionLP<TState, TAction, TValue>::getQValueRelaxation(const TState &compressed_serial_occupancy_state, typename TState::jhistory_type joint_history, typename TAction::output_type action, number t)
    {
        double weight = 0.0;

        // Go over all hidden serial state conditional to a joint_history
        for (auto hidden_serial_state : this->getWorld()->getUnderlyingProblem()->getStateSpace(t)->getAll())
        {
            //< \sum_x s(x,o) Q_MDP(x,u)
            weight += compressed_serial_occupancy_state.at(std::make_pair(hidden_serial_state, joint_history)) * std::static_pointer_cast<RelaxedValueFunction<typename TState::state_type, TState>>(this->getInitFunction())->operator()(std::make_pair(hidden_serial_state, action), t);
        }
        return weight;
    }

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int>>
    void SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedyObjective(const TState &compressed_serial_occupancy_state, IloObjective &obj, IloNumVarArray &var, number t)
    {
        // <! 1.a get variable v
        auto recover = this->getNumber(this->getVarNameWeight(0));
        number agent_id = compressed_serial_occupancy_state.getCurrentAgentId();

        //<! 1.b set coefficient of objective function "\sum_{o_i,u_i} a_i(u_i|o_i) \sum_x s(x,o_i) Q_MDP(x,u_i) + discount * v0"
        obj.setLinearCoef(var[recover], this->getWorld()->getUnderlyingProblem()->getDiscount(t));

        std::unordered_set<typename TState::jhistory_type::element_type::ihistory_type> indiv_histories;

        // Go over all joint history
        for (const auto &indiv_history : compressed_serial_occupancy_state.getIndividualHistories(agent_id))
        {
            for (const auto serial_action : this->world_->getUnderlyingProblem()->getActionSpace(t)->getAll())
            {
                recover = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(serial_action, indiv_history, agent_id));

                double res = 0;
                for (const auto &joint_history : compressed_serial_occupancy_state.getJointHistories())
                {
                    if (joint_history->getIndividualHistory(agent_id) == indiv_history)
                    {
                        //<! 1.c.5 set coefficient of variable a_i(u_i|o_i) i.e., \sum_x s(x,o_i) Q_MDP(x,u_i)
                        res += this->template getQValueRelaxation<TState>(compressed_serial_occupancy_state, joint_history, serial_action, t);
                    }
                }
                obj.setLinearCoef(var[recover], res);
            }
        }
    }

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int>>
    void SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedySawtoothBigM(const TState &compressed_serial_occupancy_state, typename TState::jhistory_type &joint_history, typename TState::state_type &next_hidden_state, typename TState::observation_type &next_observation, typename TState::jhistory_type &next_joint_history, const TState &next_one_step_uncompressed_serial_occupancy_state, double probability, double difference, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {

        number agent_id = compressed_serial_occupancy_state.getCurrentAgentId();
        // Gets the current individual history conditional on the current joint history
        auto indiv_history = joint_history->getIndividualHistory(agent_id);

        con.add(IloRange(env, -IloInfinity, this->bigM_value_));
        con[index].setLinearCoef(var[this->getNumber(this->getVarNameWeight(0))], +1.0);

        if (compressed_serial_occupancy_state.getJointHistories().find(joint_history) != compressed_serial_occupancy_state.getJointHistories().end())
        {
            // Go over all actions
            for (const auto &serial_action : this->getWorld()->getUnderlyingProblem()->getActionSpace(t)->getAll())
            {
                //<! 1.c.4 get variable a(u|o) and set constant
                con[index].setLinearCoef(var[this->getNumber(this->getVarNameIndividualHistoryDecisionRule(serial_action, indiv_history, agent_id))], -this->getQValueRealistic(compressed_serial_occupancy_state, joint_history, serial_action, next_hidden_state, next_observation, probability, difference));
            }
        }
        // <! \omega_k(x',o') * BigM
        auto VarName = this->getVarNameWeightedStateJointHistory(next_one_step_uncompressed_serial_occupancy_state, next_hidden_state, next_joint_history);
        con[index].setLinearCoef(var[this->getNumber(VarName)], this->bigM_value_);

        index++;
    }

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int>>
    void SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedySawtoothIloIfThen(const TState &compressed_serial_occupancy_state, typename TState::jhistory_type &joint_history, typename TState::state_type &next_hidden_state, typename TState::observation_type &next_observation, typename TState::jhistory_type &next_joint_history, const TState &next_one_step_uncompressed_serial_occupancy_state, double probability, double difference, IloEnv &env, IloModel &model, IloNumVarArray &var, number t)
    {
        number agent_id = compressed_serial_occupancy_state.getCurrentAgentId();
        // Gets the current individual history conditional on the current joint history
        auto indiv_history = joint_history->getIndividualHistory(agent_id);

        number recover = 0;

        IloExpr expr(env);
        //<! 1.c.1 get variable v and set coefficient of variable v
        expr = var[this->getNumber(this->getVarNameWeight(0))];

        if (compressed_serial_occupancy_state.getJointHistories().find(joint_history) != compressed_serial_occupancy_state.getJointHistories().end())
        {
            // Go over all actions
            for (const auto &serial_action : this->getWorld()->getUnderlyingProblem()->getActionSpace(t)->getAll())
            {
                //<! 1.c.4 get variable a(u|o) and set constant
                expr -= this->getQValueRealistic(compressed_serial_occupancy_state, joint_history, serial_action, next_hidden_state, next_observation, probability, difference) * var[this->getNumber(this->getVarNameIndividualHistoryDecisionRule(serial_action, indiv_history, agent_id))];
            }
        }

        // <! get variable \omega_k(x',o')
        recover = this->getNumber(this->getVarNameWeightedStateJointHistory(next_one_step_uncompressed_serial_occupancy_state, next_hidden_state, next_joint_history));
        model.add(IloIfThen(env, var[recover] > 0, expr <= 0));
    }

    // --------------------------------------------------------------------------
    // -------------  OccupancyState<BeliefState, JointHistory>  ----------------
    // --------------------------------------------------------------------------

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>, T>, int>>
    double SawtoothValueFunctionLP<TState, TAction, TValue>::getSawtoothMinimumRatio(const TState &compressed_occupancy_state,
                                                                                     typename TState::jhistory_type joint_history,
                                                                                     typename TAction::output_type action,
                                                                                     typename TState::state_type next_belief,
                                                                                     typename TState::observation_type next_observation,
                                                                                     double denominator)
    {
        assert(compressed_occupancy_state.getStatesAt(joint_history).size() == 1);

        auto under_pb = this->getWorld()->getUnderlyingProblem();
        auto factor = 0.0;

        for (const auto &belief : compressed_occupancy_state.getStatesAt(joint_history))
        {
            // Gets next belief b' = T(b,u,z)
            auto next_computed_belief = belief->expand(under_pb->getActionSpace()->joint2single(action),
                                                       under_pb->getObsSpace()->joint2single(next_observation));

            // If next computed beliefs is equal to input next belief
            if (next_computed_belief == next_belief)
            {
                // factor += s_t(b_t,o_t) * p(b_{t+1} | b_t, a)
                factor += compressed_occupancy_state.at(std::make_pair(belief, joint_history)) * belief->getProbability(under_pb->getActionSpace()->joint2single(action),
                                                                                                                        under_pb->getObsSpace()->joint2single(next_observation));
            }
        }

        return (factor / denominator);
    }

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>, T>, int>>
    double SawtoothValueFunctionLP<TState, TAction, TValue>::getQValueRelaxation(const TState &compressed_occupancy_state, typename TState::jhistory_type joint_history, typename TAction::output_type action, number t)
    {
        // assert(compressed_occupancy_state.getStatesAt(joint_history).size() == 1);
        auto weight = 0.0;
        // for (const auto &belief : compressed_occupancy_state.getStatesAt(joint_history))
        // {
        //     for (number state = 0; state < belief->getData().size(); ++state)
        //     {
        //         weight += compressed_occupancy_state.at(std::make_pair(belief, joint_history)) * belief->getData()[state] * this->getInitFunction()->operator()(std::make_pair(state, this->getWorld()->getUnderlyingProblem()->getActionSpace()->joint2single(action)), t);
        //     }
        // }
        return weight;
    }
}
