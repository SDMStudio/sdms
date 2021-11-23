#ifdef WITH_CPLEX
#include <sdm/utils/value_function/action_selection/lp/action_sawtooth_lp.hpp>
#include <sdm/utils/value_function/vfunction/tabular_value_function.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>
#include <sdm/core/state/interface/serial_interface.hpp>

#include <sdm/world/base/mpomdp_interface.hpp>
#include <sdm/world/occupancy_mdp.hpp>

namespace sdm
{
    ActionSelectionSawtoothLP::ActionSelectionSawtoothLP() {}
    ActionSelectionSawtoothLP::ActionSelectionSawtoothLP(const std::shared_ptr<SolvableByDP> &world, TypeOfResolution current_type_of_resolution, number bigM_value, TypeSawtoothLinearProgram type_of_linear_program) : ActionSelectionBase(world), DecentralizedLP(world), current_type_of_resolution_(current_type_of_resolution), type_of_linear_program_(type_of_linear_program)
    {
        this->bigM_value_ = bigM_value;
    }


    Pair<std::shared_ptr<Action>, double> ActionSelectionSawtoothLP::getGreedyActionAndValue(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, number t)
    {
        // For the Full version of Sawtooth, the support is the element of the Point Set at t+1 and the support of each element
        this->all_support = std::unordered_map<Pair<std::shared_ptr<State>, double>, std::unordered_map<std::shared_ptr<HistoryInterface>, std::vector<std::shared_ptr<State>>>>();

        //GO over all point set at t+1
        for (const auto &ostate_AND_value : std::dynamic_pointer_cast<TabularValueFunction>(vf)->getRepresentation(t + 1))
        {
            const auto &next_one_step_uncompressed_occupancy_state = ostate_AND_value.first->toOccupancyState()->getOneStepUncompressedOccupancy();

            this->all_support.emplace(ostate_AND_value, std::unordered_map<std::shared_ptr<HistoryInterface>, std::vector<std::shared_ptr<State>>>());

            // Go over all Joint History Next
            for (const auto &next_joint_history : next_one_step_uncompressed_occupancy_state->getJointHistories())
            {
                this->all_support[ostate_AND_value].emplace(next_joint_history, std::vector<std::shared_ptr<State>>());

                // Go over all Hidden State in the next one step uncomppresed occupancy state
                for (const auto next_hidden_state : next_one_step_uncompressed_occupancy_state->getBeliefAt(next_joint_history)->getStates())
                {
                    this->all_support[ostate_AND_value][next_joint_history].push_back(next_hidden_state);
                }
            }
        }
        return this->createLP(vf, state, t);
    }


    void ActionSelectionSawtoothLP::createObjectiveFunction(const std::shared_ptr<ValueFunctionInterface> &, const std::shared_ptr<State> &, IloNumVarArray &var, IloObjective &obj, number)
    {
        // <! 1.a get variable v
        auto recover = this->getNumber(this->getVarNameWeight(0));
        obj.setLinearCoef(var[recover], 1);
    }


    void ActionSelectionSawtoothLP::createVariables(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t)
    {
        try
        {
            //<! 0.b Build variables v_0 = objective variable!
            std::string VarName = this->getVarNameWeight(0);
            var.add(IloNumVar(env, -IloInfinity, IloInfinity, VarName.c_str()));
            this->setNumber(VarName, index++);

            //<! Define variables \omega_k(x',o')
            // Go over all Point Set in t+1
            for (const auto &next_state_AND_value_AND_All_next_history_AND_All_next_hidden_state : this->all_support)
            {
                const auto &ostate = next_state_AND_value_AND_All_next_history_AND_All_next_hidden_state.first.first->toOccupancyState()->getOneStepUncompressedOccupancy();

                // Go over all Joint History Next
                for (const auto &next_history_AND_All_next_hidden_state : next_state_AND_value_AND_All_next_history_AND_All_next_hidden_state.second)
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
            DecentralizedLP::createVariables(vf, state, env, var, index, t);
        }
        catch (const std::exception &exc)
        {
            std::cerr << "SawtoothLPBackup::createVariables(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
    }



    void ActionSelectionSawtoothLP::createConstraints(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloModel &model, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        auto value_function = std::dynamic_pointer_cast<ValueFunction>(vf);
        assert(value_function->getInitFunction() != nullptr);
        //<!  Build sawtooth constraints v - \sum_{u} a(u|o) * Q(k,s,o,u,y,z, diff, t  ) + \omega_k(y,<o,z>)*M <= M,  \forall k, y,<o,z>
        //<!  Build sawtooth constraints  Q(k,s,o,u,y,z, diff, t ) = (v_k - V_k) \frac{\sum_{x} s(x,o) * p(x,u,z,y)}}{s_k(y,<o,z>)},  \forall a(u|o)

        try
        {
            auto compressed_occupancy_state = state->toOccupancyState();

            if (value_function->getSupport(t + 1).empty())
            {
                this->createInitialConstraints(value_function, state, env, con, var, index, t);
            }
            else
            {
                // Go over all points in the point set at t+1
                for (const auto &next_state_AND_value_AND_All_next_history_AND_All_next_hidden_state : this->all_support)
                {
                    const auto &ostate = next_state_AND_value_AND_All_next_history_AND_All_next_hidden_state.first.first->toOccupancyState();
                    const auto &next_one_step_uncompressed_occupancy_state = ostate->toOccupancyState()->getOneStepUncompressedOccupancy();

                    // Compute the difference i.e. (v_k - V_k)
                    double difference = value_function->computeDifference(next_state_AND_value_AND_All_next_history_AND_All_next_hidden_state.first, t);

                    // Go over all joint histories in over the support of next_one_step_uncompressed_occupancy_state
                    for (const auto &next_history_AND_All_next_hidden_state : next_state_AND_value_AND_All_next_history_AND_All_next_hidden_state.second)
                    {
                        const auto &next_history = next_history_AND_All_next_hidden_state.first->toJointHistory();

                        // Go over all Hidden State in the Belief for a precise Joint History
                        for (const auto &next_hidden_state : next_history_AND_All_next_hidden_state.second)
                        {
                            //Determine denominator of the sawtooth ratio
                            double denominator = next_one_step_uncompressed_occupancy_state->getProbability(next_history, next_hidden_state);

                            // Determine the next Joint observation thanks to the next joint history
                            auto next_joint_observation = std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(next_history->getLastObservation())

                            // We search for the joint_history which allow us to obtain the current next_history conditionning to the next joint observation
                            switch (this->current_type_of_resolution_)
                            {
                            case TypeOfResolution::BigM:
                                this->createSawtoothBigM(value_function, state, nullptr, next_hidden_state, next_joint_observation, next_history, next_one_step_uncompressed_occupancy_state, denominator, difference, env, con, var, index, t);
                                break;
                            case TypeOfResolution::IloIfThenResolution:
                                this->createSawtoothIloIfThen(value_function, state, nullptr, next_hidden_state, next_joint_observation, next_history, next_one_step_uncompressed_occupancy_state, denominator, difference, env, model, var, t);
                                break;
                            }
                        }
                    }
                    this->createOmegaConstraints(next_state_AND_value_AND_All_next_history_AND_All_next_hidden_state.first, env, con, var, index);
                }
            }
        }
        catch (const std::exception &exc)
        {
            std::cerr << "ActionSelectionSawtoothLP::createConstraints(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
    }

    void ActionSelectionSawtoothLP::createOmegaConstraints(const Pair<std::shared_ptr<State>, double> &state_AND_value, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index)
    {
        const auto &ostate = state_AND_value.first->toOccupancyState()->getOneStepUncompressedOccupancy();

        number recover = 0;

        // Build constraint \sum{x',o'} \omega_k(x',o') = 1
        con.add(IloRange(env, 1.0, 1.0));

        // Go over all joint histories in over the support of next_one_step_uncompressed_occupancy_state
        for (const auto &next_history_AND_All_next_hidden_state : this->all_support[state_AND_value])
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

    void ActionSelectionSawtoothLP::createInitialConstraints(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        auto value_function = std::dynamic_pointer_cast<ValueFunction>(vf);
        auto underlying_problem = ActionSelectionBase::world_->getUnderlyingProblem();

        number recover = 0;

        con.add(IloRange(env, -IloInfinity, 0));
        con[index].setLinearCoef(var[this->getNumber(this->getVarNameWeight(0))], +1.0);

        // Go over all actions
        for (const auto &action : *underlying_problem->getActionSpace(t))
        {
            for (const auto &joint_history : state->toOccupancyState()->getJointHistories())
            {
                //<! 1.c.4 get variable a(u|o) and set constant
                recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action->toAction(), joint_history));

                con[index].setLinearCoef(var[recover], -this->getQValueRelaxation(value_function, state, joint_history, action->toAction(), t));
            }
        }
        index++;
    }

    void ActionSelectionSawtoothLP::createSawtoothBigM(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> &, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double denominator, double difference, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        try
        {
            auto underlying_problem = ActionSelectionBase::world_->getUnderlyingProblem();
            number recover = 0;

            con.add(IloRange(env, -IloInfinity, this->bigM_value_));
            con[index].setLinearCoef(var[this->getNumber(this->getVarNameWeight(0))], +1.0);

            // Go over all actions
            for (const auto &action : *underlying_problem->getActionSpace(t))
            {
                for (const auto &joint_history : state->toOccupancyState()->getJointHistories())
                {
                    //<! 1.c.4 get variable a(u|o) and set constant
                    recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action->toAction(), joint_history));

                    //<! 1.c.4 get variable a(u|o) and set constant
                    con[index].setLinearCoef(var[recover], -this->computeSawtooth(vf, state, action->toAction(), joint_history, next_hidden_state, next_observation, next_joint_history, denominator, difference, t));
                }
            }
            // <! \omega_k(x',o') * BigM
            recover = this->getNumber(this->getVarNameWeightedStateJointHistory(next_state, next_hidden_state, next_joint_history));
            con[index].setLinearCoef(var[recover], this->bigM_value_);

            index++;
        }
        catch (const std::exception &exc)
        {
            // catch anything thrown within try block that derives from std::exception
            std::cerr << "SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedySawtoothBigM(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
    }

    void ActionSelectionSawtoothLP::createSawtoothIloIfThen(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> &, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double denominator, double difference, IloEnv &env, IloModel &model, IloNumVarArray &var, number t)
    {
        try
        {
            auto underlying_problem = ActionSelectionBase::world_->getUnderlyingProblem();
            number recover = 0;

            IloExpr expr(env);
            //<! 1.c.1 get variable v and set coefficient of variable v
            expr = var[this->getNumber(this->getVarNameWeight(0))];

            // Go over all actions
            for (const auto &action : *underlying_problem->getActionSpace(t))
            {
                for (const auto &joint_history : state->toOccupancyState()->getJointHistories())
                {
                    recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action->toAction(), joint_history));

                    //<! 1.c.4 get variable a(u|o) and set constant
                    expr -= var[recover] * this->computeSawtooth(vf, state, action->toAction(), joint_history, next_hidden_state, next_observation, next_joint_history, denominator, difference, t);
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



}

#endif