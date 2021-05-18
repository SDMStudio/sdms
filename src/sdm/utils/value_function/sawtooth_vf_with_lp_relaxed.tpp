namespace sdm
{

    template <typename TState, typename TAction, typename TValue>
    SawtoothValueFunctionLPRelaxed<TState, TAction, TValue>::SawtoothValueFunctionLPRelaxed()
    {
        this->setTStateType(ONE_STEP_UNCOMPRESSED);
    }

    template <typename TState, typename TAction, typename TValue>
    SawtoothValueFunctionLPRelaxed<TState, TAction, TValue>::SawtoothValueFunctionLPRelaxed(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, number horizon, std::shared_ptr<Initializer<TState, TAction>> initializer)
        : DecentralizedConstraintsLP<TState, TAction, TValue>(problem),
          SawtoothValueFunction<TState, TAction, TValue>(problem, horizon, initializer)
    {
        this->setTStateType(ONE_STEP_UNCOMPRESSED);
    }

    template <typename TState, typename TAction, typename TValue>
    SawtoothValueFunctionLPRelaxed<TState, TAction, TValue>::SawtoothValueFunctionLPRelaxed(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, number horizon, TValue default_value)
        : SawtoothValueFunctionLPRelaxed<TState, TAction, TValue>(problem, horizon, std::make_shared<ValueInitializer<TState, TAction>>(default_value))
    {
    }

    template <typename TState, typename TAction, typename TValue>
    TAction SawtoothValueFunctionLPRelaxed<TState, TAction, TValue>::getBestAction(const TState &compressed_occupancy_state, number t)
    {
        return this->getGreedySawtooth(compressed_occupancy_state,t).first;
    }

    template <typename TState, typename TAction, typename TValue>
    void SawtoothValueFunctionLPRelaxed<TState, TAction, TValue>::updateValueAt(const TState &occupancy_state, number t)
    {
        double cub = this->getGreedySawtooth(occupancy_state,t).second;
        switch (this->ctype)
        {
            case TState_t::FULLY_UNCOMPRESSED:
                MappedValueFunction<TState, TAction, TValue>::updateValueAt(*occupancy_state.getFullyUncompressedOccupancy(), t, cub);
                break;
            case TState_t::ONE_STEP_UNCOMPRESSED:
                MappedValueFunction<TState, TAction, TValue>::updateValueAt(*occupancy_state.getOneStepUncompressedOccupancy(), t, cub);
                break;
            default:
                MappedValueFunction<TState, TAction, TValue>::updateValueAt(occupancy_state, t, cub);
                break;
        }
    }

    template <typename TState, typename TAction, typename TValue>
    TValue SawtoothValueFunctionLPRelaxed<TState, TAction, TValue>::getValueAt(const TState &occupancy_state, number t)
    {
        switch (this->ctype)
        {
        case TState_t::FULLY_UNCOMPRESSED:
            return SawtoothValueFunction<TState, TAction, TValue>::getValueAt(*occupancy_state.getFullyUncompressedOccupancy(), t);
        case TState_t::ONE_STEP_UNCOMPRESSED:
            return SawtoothValueFunction<TState, TAction, TValue>::getValueAt(*occupancy_state.getOneStepUncompressedOccupancy(), t);
        default:
            return SawtoothValueFunction<TState, TAction, TValue>::getValueAt(occupancy_state, t);
        }
    }

    template <typename TState, typename TAction, typename TValue>
    Pair<TAction,double> SawtoothValueFunctionLPRelaxed<TState, TAction, TValue>::getGreedySawtooth(const TState &compressed_occupancy_state, number t)
    {
        TAction max_decision_rule;
        Pair<TAction,double> decision_rule_AND_value;
        double max = - std::numeric_limits<double>::max();

        // Go over all Point Set in t+1
        if( this->representation[t + 1].empty() ){
            decision_rule_AND_value = this->greedySawtooth(compressed_occupancy_state, std::make_pair(compressed_occupancy_state,0), t);
            max_decision_rule = decision_rule_AND_value.first;
            max = decision_rule_AND_value.second;
        }
        else
        {
            for (const auto &next_one_step_uncompressed_occupancy_state_AND_upper_bound : this->representation[t + 1])
            {
                decision_rule_AND_value = this->greedySawtooth(compressed_occupancy_state, next_one_step_uncompressed_occupancy_state_AND_upper_bound, t);

                if (decision_rule_AND_value.second > max)
                {
                    max_decision_rule = decision_rule_AND_value.first;
                    max = decision_rule_AND_value.second;
                }
            }
        }

        return std::make_pair(max_decision_rule, max);
    }

    template <typename TState, typename TAction, typename TValue>
    Pair<TAction,double> SawtoothValueFunctionLPRelaxed<TState, TAction, TValue>::greedySawtooth(const TState &occupancy_state,const Pair<TState,double>& next_one_step_uncompressed_occupancy_state_AND_upper_bound, number t,double limite_inf, double limite_sup)
    {
        number index = 0;

        //<! tracking variables
        std::string VarName;

        TAction a;
        double cub;
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
            this->setGreedyVariables(occupancy_state, env, var,limite_inf,limite_sup, t);

            // 1. Build objective function \sum_{o,u} a(u|o) \sum_x s(x,o) Q_MDP(x,u) - discount * v
            this->template setGreedyObjective<TState>(occupancy_state, obj, var, t);

            //<! 3.a Build sawtooth constraints v <= (V_k - v_k) * \frac{\sum_{o,u} a(u|o)\sum_{x,z_} s(x,o)*p(x,u,z_,x_)}}{s_k(x_,o_)} ,\forall k, x_,o_
            this->template setGreedySawtooth<TState>(occupancy_state,next_one_step_uncompressed_occupancy_state_AND_upper_bound, model, env, con, var, index, t);

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

        return std::make_pair(a, cub);
    }

    template <typename TState, typename TAction, typename TValue>
    void SawtoothValueFunctionLPRelaxed<TState, TAction, TValue>::setGreedyVariables(const TState &occupancy_state, IloEnv &env, IloNumVarArray &var, double /*limite_inf*/, double /*limite_sup*/, number t)
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

        this->template setDecentralizedVariables<TState>(occupancy_state, env, var, index, t);
    }

    template <typename TState, typename TAction, typename TValue>
    double SawtoothValueFunctionLPRelaxed<TState, TAction, TValue>::getQValueRealistic(const TState &compressed_occupancy_state, typename TState::jhistory_type joint_history, typename TAction::output_type action, typename TState::state_type next_hidden_state, typename TState::observation_type next_observation, double denominator, double difference)
    {
        return difference * this->template getSawtoothMinimumRatio<TState>(compressed_occupancy_state, joint_history, action, next_hidden_state, next_observation, denominator);
    }

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_any<T, OccupancyState<>, OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>>::value, int>>
    void SawtoothValueFunctionLPRelaxed<TState, TAction, TValue>::setGreedyObjective(const TState &compressed_occupancy_state, IloObjective &obj, IloNumVarArray &var, number t)
    {
        // <! 1.a get variable v
        auto recover = this->getNumber(this->getVarNameWeight(0));

        //<! 1.b set coefficient of objective function "\sum_{o,u} a(u|o) \sum_x s(x,o) Q_MDP(x,u) - discount * v0"
        obj.setLinearCoef(var[recover],this->getWorld()->getUnderlyingProblem()->getDiscount(t));

        // Go over all joint history
        for (const auto &joint_history : compressed_occupancy_state.getJointHistories())
        {
            // Go over all action
            for (const auto &action : this->getWorld()->getUnderlyingProblem()->getActionSpace()->getAll())
            {
                //<! 1.c.4 get variable a(u|o)
                recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action, joint_history));

                //<! 1.c.5 set coefficient of variable a(u|o) i.e., \sum_x s(x,o) Q_MDP(x,u)
                obj.setLinearCoef(var[recover], this->template getQValueRelaxation<TState>(compressed_occupancy_state, joint_history, action, t));
            }
        }
    }

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<>, T>, int>>
    double SawtoothValueFunctionLPRelaxed<TState, TAction, TValue>::getQValueRelaxation(const TState &compressed_occupancy_state, typename TState::jhistory_type joint_history, typename TAction::output_type action, number t)
    {
        auto weight = 0.0;
        // Go over all state conditionning a joint history
        for (auto x : compressed_occupancy_state.getStatesAt(joint_history))
        {
            // \sum_{x} s(x,o) * Q_MDP(x,u)
            weight += compressed_occupancy_state.at(std::make_pair(x, joint_history)) * std::static_pointer_cast<State2OccupancyValueFunction<typename TState::state_type, TState>>(this->getInitFunction())->getQValueAt(x, this->getWorld()->getUnderlyingProblem()->getActionSpace()->joint2single(action), t);
        }
        return weight;
    }

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_any<T, OccupancyState<>, OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>>::value, int> >
    void SawtoothValueFunctionLPRelaxed<TState, TAction, TValue>::setGreedySawtooth(const TState& compressed_occupancy_state,const Pair<TState,double>& next_one_step_uncompressed_occupancy_state_AND_upper_bound, IloModel &, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        //<!  Build sawtooth constraints v - \sum_{u} a(u|o) * Q(k,s,o,u,y,z, diff, t  ) + \omega_k(y,<o,z>)*M <= M,  \forall k, y,<o,z>
        //<!  Build sawtooth constraints  Q(k,s,o,u,y,z, diff, t ) = (v_k - V_k) \frac{\sum_{x} s(x,o) * p(x,u,z,y)}}{s_k(y,<o,z>)},  \forall a(u|o)

        assert(this->getInitFunction() != nullptr);

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

            con.add(IloRange(env, -IloInfinity, 0));
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
            index++;
        }
    }

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<>, T>, int>>
    double SawtoothValueFunctionLPRelaxed<TState, TAction, TValue>::getSawtoothMinimumRatio(const TState &compressed_occupancy_state, typename TState::jhistory_type joint_history, typename TAction::output_type action, typename TState::state_type next_hidden_state, typename TState::observation_type next_observation, double denominator)
    {
        auto factor = 0.0;

        // Go over all state conditionning to a joint history
        for (const auto &hidden_state : compressed_occupancy_state.getStatesAt(joint_history))
        {
            // \sum_{x} s(x,o) * p_{x,u,z',x'}
            factor += compressed_occupancy_state.at(std::make_pair(hidden_state, joint_history)) * this->getWorld()->getUnderlyingProblem()->getObsDynamics()->getDynamics(hidden_state, this->getWorld()->getUnderlyingProblem()->getActionSpace()->joint2single(action), this->getWorld()->getUnderlyingProblem()->getObsSpace()->joint2single(next_observation), next_hidden_state);
        }

        return factor / denominator;    
    }


    // --------------------------------------------------------------------------
    // -------------  SerializedOccupancyState<TState, JointHistory>  ----------------
    // --------------------------------------------------------------------------

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int>>
    void SawtoothValueFunctionLPRelaxed<TState, TAction, TValue>::setGreedySawtooth(const TState &compressed_serial_occupancy_state,const Pair<TState,double>& next_one_step_uncompressed_serial_occupancy_state_AND_upper_bound, IloModel &, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        number agent_id = compressed_serial_occupancy_state.getCurrentAgentId();

        auto &next_one_step_uncompressed_serial_occupancy_state = next_one_step_uncompressed_serial_occupancy_state_AND_upper_bound.first;
        
        auto initial_upper_bound = this->getInitFunction()->operator()(next_one_step_uncompressed_serial_occupancy_state, t + 1);
        auto difference = next_one_step_uncompressed_serial_occupancy_state_AND_upper_bound.second -initial_upper_bound;
        
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

            // Gets the current individual history conditional on the current joint history
            auto indiv_history = joint_history->getIndividualHistory(agent_id);

            con.add(IloRange(env, -IloInfinity, 0));
            con[index].setLinearCoef(var[this->getNumber(this->getVarNameWeight(0))], +1.0);

            if (compressed_serial_occupancy_state.getJointHistories().find(joint_history) != compressed_serial_occupancy_state.getJointHistories().end())
            {
                // Go over all actions
                for (const auto &serial_action : this->getWorld()->getUnderlyingProblem()->getActionSpace(t)->getAll())
                {
                    //<! 1.c.4 get variable a(u|o) and set constant
                    con[index].setLinearCoef(var[this->getNumber(this->getVarNameIndividualHistoryDecisionRule(serial_action, indiv_history, agent_id))], -this->getQValueRealistic(compressed_serial_occupancy_state, joint_history, serial_action, next_hidden_serial_state, next_observation, probability, difference));
                }
            }
            index++;
        }
    }

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int>>
    double SawtoothValueFunctionLPRelaxed<TState, TAction, TValue>::getSawtoothMinimumRatio(const TState &compressed_serial_occupancy_state, typename TState::jhistory_type joint_history, typename TAction::output_type action, typename TState::state_type next_hidden_serial_state, typename TState::observation_type next_observation, double denominator)
    {
        auto factor = 0.0;

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
    double SawtoothValueFunctionLPRelaxed<TState, TAction, TValue>::getQValueRelaxation(const TState &compressed_serial_occupancy_state, typename TState::jhistory_type joint_history, typename TAction::output_type action, number t)
    {
        auto weight = 0.0;

        // Go over all hidden serial state conditional to a joint_history
        for (auto hidden_serial_state :  this->getWorld()->getUnderlyingProblem()->getStateSpace(t)->getAll())
        {
            //< \sum_x s(x,o) Q_MDP(x,u)
            weight += compressed_serial_occupancy_state.at(std::make_pair(hidden_serial_state, joint_history)) * std::static_pointer_cast<State2OccupancyValueFunction<typename TState::state_type, TState>>(this->getInitFunction())->getQValueAt(hidden_serial_state, action, t);
        }
        return weight;
    }

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int>>
    void SawtoothValueFunctionLPRelaxed<TState, TAction, TValue>::setGreedyObjective(const TState &compressed_serial_occupancy_state, IloObjective &obj, IloNumVarArray &var, number t)
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
                for (const auto &joint_history : compressed_serial_occupancy_state.getJointHistoryOverIndividualHistories(agent_id,indiv_history))
                {
                    //<! 1.c.5 set coefficient of variable a_i(u_i|o_i) i.e., \sum_x s(x,o_i) Q_MDP(x,u_i)
                    res += this->template getQValueRelaxation<TState>(compressed_serial_occupancy_state, joint_history, serial_action, t);
                }
                obj.setLinearCoef(var[recover],res);
            }
        }
    }


    // --------------------------------------------------------------------------
    // -------------  OccupancyState<BeliefState, JointHistory>  ----------------
    // --------------------------------------------------------------------------

    template <typename TState, typename TAction, typename TValue>
    template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>, T>, int>>
    double SawtoothValueFunctionLPRelaxed<TState, TAction, TValue>::getSawtoothMinimumRatio(const TState &compressed_occupancy_state,
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
    double SawtoothValueFunctionLPRelaxed<TState, TAction, TValue>::getQValueRelaxation(const TState &compressed_occupancy_state, typename TState::jhistory_type joint_history, typename TAction::output_type action, number t)
    {
        assert(compressed_occupancy_state.getStatesAt(joint_history).size() == 1);
        auto weight = 0.0;
        for (const auto &belief : compressed_occupancy_state.getStatesAt(joint_history))
        {
            for (number state = 0; state < belief->getData().size(); ++state)
            {
                weight += compressed_occupancy_state.at(std::make_pair(belief, joint_history)) * belief->getData()[state] * std::static_pointer_cast<State2OccupancyValueFunction<number, TState>>(this->getInitFunction())->getQValueAt(state, this->getWorld()->getUnderlyingProblem()->getActionSpace()->joint2single(action), t);
            }
        }
        return weight;
    }
}
