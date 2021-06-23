#include <sdm/utils/value_function/backup/maxplan_lp_backup.hpp>
#include <sdm/world/base/mpomdp_interface.hpp>


namespace sdm
{
    MaxPlanLPBackup::MaxPlanLPBackup() {} 
    MaxPlanLPBackup::MaxPlanLPBackup(const std::shared_ptr<SolvableByHSVI>& world) : MaxPlanBackup(world), DecentralizedConstraintsLP(world){}
    
    std::shared_ptr<State> MaxPlanLPBackup::backup(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t)
    {
        auto pair_hyperplan_action = this->getBestActionAndMaxHyperplan(vf,state,t);
        return this->setHyperplan(vf,state, pair_hyperplan_action.first->toBelief(), pair_hyperplan_action.second, t);
    }
  
    std::shared_ptr<Action> MaxPlanLPBackup::getBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t)
    {
        return this->getBestActionAndMaxHyperplan(vf,state,t).second;
    }
    Pair<std::shared_ptr<State>,std::shared_ptr<Action>> MaxPlanLPBackup::getBestActionAndMaxHyperplan(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t)
    {
        std::shared_ptr<State> next_hyperplan;
        std::shared_ptr<Action> max_decision_rule, joint_decision_rule;
        double max = -std::numeric_limits<double>::max(), value;

        for (const auto &hyperplan : vf->getSupport(t + 1))
        {
            this->tmp_representation = hyperplan->toBelief()->getVectorInferface();
            auto pair_action_value = this->getGreedy(state->toOccupancyState(), t);

            joint_decision_rule = pair_action_value.first;
            value = pair_action_value.second;

            if (value > max)
            {
                max_decision_rule = joint_decision_rule;
                next_hyperplan = hyperplan;
                max = value;
            }
        }
        return std::make_pair(next_hyperplan,max_decision_rule);
    }

    Pair<std::shared_ptr<Action>,double> MaxPlanLPBackup::getGreedy(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, number t)
    {
        //Identifiant, compteur des contraintes
        number index = 0;

        //<! greedy decision, initialization
        std::shared_ptr<Action> action;
        double value;

        IloEnv env;
        try
        {
            IloModel model(env);

            // Init the model
            IloRangeArray con(env);
            IloNumVarArray var(env);

            IloObjective obj = IloMaximize(env);

            ///////  BEGIN CORE CPLEX Code  ///////

            // 0. Build variables a(u|o), a_i(u_i|o_i)
            this->setGreedyVariables(occupancy_state, env, var, t);

            // 1. Build objective function \sum_{o,u} A(u|o) \sum_x s(x,o)  [ r(x,u) + \gamma \sum_{x_,z_} P(x_,z_|x,u) * \hyperplan_i(x_,o_) ]
            this->setGreedyObjective(occupancy_state, var, obj, t);

            // 3. Build decentralized control constraints [  a(u|o) >= \sum_i a_i(u_i|o_i) + 1 - n ] ---- and ---- [ a(u|o) <= a_i(u_i|o_i) ]
            this->setDecentralizedConstraints(occupancy_state, env, con, var, index, t);

            ///////  END CORE  CPLEX Code ///////
            model.add(obj);
            model.add(con);
            IloCplex cplex(model);
            cplex.setOut(env.getNullStream());
            cplex.setWarning(env.getNullStream());

            // Optimize the problem and obtain solution
            if (!cplex.solve())
            {
                env.error() << "Failed to optimize MILP" << std::endl;
                // cplex.exportModel("lb_bellman_op.lp");
                // system("cat lb_bellman_op.lp");
                // throw(-1);
            }
            else
            {
                value = cplex.getObjValue();
                action = this->getDecentralizedVariables(cplex, var, occupancy_state, t);
            }
        }
        catch (IloException &e)
        {
            //"Concert exception caught: " << e << std::endl;
        }
        catch (const std::exception &exc)
        {
            // catch anything thrown within try block that derives from std::exception
            std::cerr << "Non-Concert exception caught: " << exc.what() << std::endl;
        }

        env.end();

        return std::make_pair(action,value);
    }

    void MaxPlanLPBackup::setGreedyVariables(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, IloEnv &env, IloNumVarArray &var, number t)
    {
        //<! counter for constraints
        number index = 0;

        //<! tracking variables
        std::string VarName;

        // re-init the set of variable names
        this->variables.clear();

        //<! Build variables v_size = objective variable!
        VarName = this->getVarNameWeight(0);
        var.add(IloNumVar(env, -IloInfinity, +IloInfinity, VarName.c_str()));
        this->setNumber(VarName, index++);

        //<! set decentralized decision rule variables
        this->setDecentralizedVariables(occupancy_state, env, var, index, t);
    }

    void MaxPlanLPBackup::setGreedyObjective(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, IloNumVarArray &var, IloObjective &obj, number t)
    {
        auto under_pb = std::dynamic_pointer_cast<MPOMDPInterface>(MaxPlanBackup::world_->getUnderlyingProblem());

        number recover = 0;
        double weight = 0.0, factor = 0.0;

        for (const auto &joint_history : occupancy_state->getJointHistories())
        {
            for (const auto &action : *under_pb->getActionSpace(t))
            {
                weight = 0.0;

                for (const auto &hidden_state : *under_pb->getStateSpace(t))
                {
                    //<! 1.a compute factor
                    factor = under_pb->getReward(hidden_state->toState(), action->toAction(),t);

                    if (t < under_pb->getHorizon() - 1)
                    {
                        for (const auto &next_hidden_state : under_pb->getReachableStates(hidden_state->toState(), action->toAction(),t))
                        {
                            for (const auto &next_observation : under_pb->getReachableObservations(hidden_state->toState(), action->toAction(), next_hidden_state->toState(),t))
                            {
                                auto joint_observation = std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(next_observation);

                                auto joint_history_next = joint_history->expand(joint_observation)->toJointHistoryTree();
                                factor += under_pb->getDynamics(hidden_state->toState(), action->toAction(),next_hidden_state,next_observation,t) * this->tmp_representation->getValueAt(occupancy_state->HiddenStateAndJointHistoryToState(next_hidden_state, joint_history_next));
                            }
                        }
                    }
                    weight += occupancy_state->getProbability(occupancy_state->HiddenStateAndJointHistoryToState(hidden_state->toState(), joint_history)) * factor * under_pb->getDiscount(t);
                }

                //<! 1.b get variable a(u|o)
                recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action->toAction(), joint_history));

                //<! 1.c set coefficient of variable a(u|o) i.e., s(x,o)  [ r(x,u) + \gamma \sum_{x_,z_} P(x_,z_|x,u) * \hyperplan_i(x_,o_)  ]
                obj.setLinearCoef(var[recover], weight);
            } // for all u
        }   // for all o
    }
}
