#ifdef WITH_CPLEX
#include <sdm/utils/value_function/action_selection/action_sawtooth_lp.hpp>
#include <sdm/utils/value_function/vfunction/tabular_value_function.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>
#include <sdm/core/state/interface/serial_interface.hpp>

#include <sdm/world/base/mpomdp_interface.hpp>
#include <sdm/world/occupancy_mdp.hpp>

namespace sdm
{
    ActionSelectionSawtoothLP::ActionSelectionSawtoothLP() {}
    ActionSelectionSawtoothLP::ActionSelectionSawtoothLP(const std::shared_ptr<SolvableByHSVI> &world, TypeOfResolution current_type_of_resolution, number bigM_value, TypeSawtoothLinearProgram type_of_linear_program) : ActionSelectionBase(world), DecentralizedLP(world), current_type_of_resolution_(current_type_of_resolution), type_of_linear_program_(type_of_linear_program)
    {
        this->bigM_value_ = bigM_value;
    }

    Pair<std::shared_ptr<Action>, double> ActionSelectionSawtoothLP::selectBestActionFull(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t)
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

    Pair<std::shared_ptr<Action>, double> ActionSelectionSawtoothLP::selectBestActionRelaxedV2(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t)
    {
        // For the Relaxed version 2 of Sawtooth, the support is only the support of each point set at t+1.
        //Consequently, we create a LP problem for each point set at t+1

        std::shared_ptr<Action> best_action;
        double min_value = std::numeric_limits<double>::max();

        if (vf->getSupport(t + 1).empty())
        {
            // Resolution of the problem when the support of Point Set is empty
            this->all_support = std::unordered_map<Pair<std::shared_ptr<State>, double>, std::unordered_map<std::shared_ptr<HistoryInterface>, std::vector<std::shared_ptr<State>>>>();
            auto best_action_AND_value = this->createLP(vf, state, t);

            best_action = best_action_AND_value.first;
            min_value = best_action_AND_value.second;
        }
        else
        {

            //GO over all point set at t+1
            for (const auto &ostate_AND_value : std::dynamic_pointer_cast<TabularValueFunction>(vf)->getRepresentation(t + 1))
            {
                const auto &next_one_step_uncompressed_occupancy_state = ostate_AND_value.first->toOccupancyState()->getOneStepUncompressedOccupancy();

                //Initialiaze the support
                this->all_support = std::unordered_map<Pair<std::shared_ptr<State>, double>, std::unordered_map<std::shared_ptr<HistoryInterface>, std::vector<std::shared_ptr<State>>>>();
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

                //Resolve a LP problem for a precise point set
                auto [action, value] = this->createLP(vf, state, t);

                if (min_value > value)
                {
                    min_value = value;
                    best_action = action;
                }
            }
        }
        return {best_action, min_value};
    }

    Pair<std::shared_ptr<Action>, double> ActionSelectionSawtoothLP::selectBestActionRelaxed(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t)
    {
        // For the Relaxed version of Sawtooth, the support is only one support of each point set at t+1.
        //Consequently, we create a LP problem for each point set at t+1 and each support of this point.

        std::shared_ptr<Action> best_action;
        double min_value = std::numeric_limits<double>::max();

        if (vf->getSupport(t + 1).empty())
        {
            // Resolution of the problem when the support of Point Set is empty
            this->all_support = std::unordered_map<Pair<std::shared_ptr<State>, double>, std::unordered_map<std::shared_ptr<HistoryInterface>, std::vector<std::shared_ptr<State>>>>();
            auto best_action_AND_value = this->createLP(vf, state, t);

            best_action = best_action_AND_value.first;
            min_value = best_action_AND_value.second;
        }
        else
        {
            //GO over all point set at t+1
            for (const auto &ostate_AND_value : std::dynamic_pointer_cast<TabularValueFunction>(vf)->getRepresentation(t + 1))
            {
                const auto &next_one_step_uncompressed_occupancy_state = ostate_AND_value.first->toOccupancyState()->getOneStepUncompressedOccupancy();

                double max_value_support = -std::numeric_limits<double>::max();
                std::shared_ptr<Action> best_action_support;

                // Go over all Joint History Next
                for (const auto &next_joint_history : next_one_step_uncompressed_occupancy_state->getJointHistories())
                {
                    // Go over all Hidden State in the next one step uncomppresed occupancy state
                    for (const auto next_hidden_state : next_one_step_uncompressed_occupancy_state->getBeliefAt(next_joint_history)->getStates())
                    {
                        //Initialize the support for a precise point set and a precise support of this point.
                        this->all_support = std::unordered_map<Pair<std::shared_ptr<State>, double>, std::unordered_map<std::shared_ptr<HistoryInterface>, std::vector<std::shared_ptr<State>>>>();

                        //Emplace the information for the LP
                        this->all_support.emplace(ostate_AND_value, std::unordered_map<std::shared_ptr<HistoryInterface>, std::vector<std::shared_ptr<State>>>());
                        this->all_support[ostate_AND_value].emplace(next_joint_history, std::vector<std::shared_ptr<State>>());
                        this->all_support[ostate_AND_value][next_joint_history].push_back(next_hidden_state);

                        auto [action, value] = this->createLP(vf, state, t);

                        // We take the best action with the minimum value
                        if (max_value_support < value)
                        {
                            max_value_support = value;
                            best_action_support = action->toAction();
                        }
                    }
                }

                if (min_value > max_value_support)
                {
                    min_value = max_value_support;
                    best_action = best_action_support;
                }
            }
        }
        return {best_action, min_value};
    }

    Pair<std::shared_ptr<Action>, double> ActionSelectionSawtoothLP::getGreedyActionAndValue(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t)
    {
        switch (this->type_of_linear_program_)
        {
        case TypeSawtoothLinearProgram::PLAIN_SAWTOOTH_LINER_PROGRAMMING:
            return this->selectBestActionFull(vf, state, t);
            break;
        case TypeSawtoothLinearProgram::RELAXED_SAWTOOTH_LINER_PROGRAMMING:
            return this->selectBestActionRelaxed(vf, state, t);
            break;
        case TypeSawtoothLinearProgram::RELAXED_V2_SAWTOOTH_LINER_PROGRAMMING:
            return this->selectBestActionRelaxedV2(vf, state, t);
            break;

        default:
            break;
        }
    }

    // ************************************************************************
    // ************************************************************************
    // ******************* Create Variable for LP *****************************
    // ************************************************************************
    // ************************************************************************

    void ActionSelectionSawtoothLP::createVariables(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t)
    {
        try
        {
            this->variables.clear();

            //<! 0.b Build variables v_0 = objective variable!
            this->createObjectiveVariable(vf, state, env, var, index, t);

            //<! Define variables \omega_k(x',o')
            this->createOmegaVariable(env, var, index);

            // Create Decentralized Variables
            this->createDecentralizedVariables(vf, state, env, var, index, t);
        }
        catch (const std::exception &exc)
        {
            std::cerr << "SawtoothLPBackup::createVariables(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
    }

    void ActionSelectionSawtoothLP::createObjectiveVariable(const std::shared_ptr<ValueFunction> &, const std::shared_ptr<State> &, IloEnv &env, IloNumVarArray &var, number &index, number)
    {
        //<! 0.b Build variables v_0 = objective variable!
        std::string VarName = this->getVarNameWeight(0);
        var.add(IloNumVar(env, -IloInfinity, IloInfinity, VarName.c_str()));
        this->setNumber(VarName, index++);
    }

    void ActionSelectionSawtoothLP::createOmegaVariable(IloEnv &env, IloNumVarArray &var, number &index)
    {
        //<! Define variables \omega_k(x',o')

        std::string VarName;

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
    }

    // ************************************************************************
    // ************************************************************************
    // ******************* Create Constraint for LP ***************************
    // ************************************************************************
    // ************************************************************************

    void ActionSelectionSawtoothLP::createConstraints(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloModel &model, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        assert(vf->getInitFunction() != nullptr);

        //<!  Build sawtooth constraints v - \sum_{u} a(u|o) * Q(k,s,o,u,y,z, diff, t  ) + \omega_k(y,<o,z>)*M <= M,  \forall k, y,<o,z>
        //<!  Build sawtooth constraints  Q(k,s,o,u,y,z, diff, t ) = (v_k - V_k) \frac{\sum_{x} s(x,o) * p(x,u,z,y)}}{s_k(y,<o,z>)},  \forall a(u|o)

        try
        {
            auto compressed_occupancy_state = state->toOccupancyState();

            if (vf->getSupport(t + 1).empty())
            {
                this->createInitialConstraints(vf, state, env, con, var, index, t);
            }
            else
            {
                // Go over all points in the point set at t+1
                for (const auto &next_state_AND_value_AND_All_next_history_AND_All_next_hidden_state : this->all_support)
                {
                    const auto &ostate = next_state_AND_value_AND_All_next_history_AND_All_next_hidden_state.first.first->toOccupancyState();
                    const auto &next_one_step_uncompressed_occupancy_state = ostate->toOccupancyState()->getOneStepUncompressedOccupancy();

                    // Compute the difference i.e. (v_k - V_k)
                    double difference = this->computeDifference(vf, next_state_AND_value_AND_All_next_history_AND_All_next_hidden_state.first, t);

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
                            auto next_joint_observation = this->determineNextJointObservation(next_history, t);

                            this->createConstraintsKnowingInformation(vf, state, nullptr, next_hidden_state, next_joint_observation, next_history, next_one_step_uncompressed_occupancy_state, denominator, difference, env, model, con, var, index, t);
                        }
                    }
                    this->createOmegaConstraints(next_state_AND_value_AND_All_next_history_AND_All_next_hidden_state.first, env, con, var, index);
                }
            }
            this->createDecentralizedConstraints(vf, state, env, con, var, index, t);
        }
        catch (const std::exception &exc)
        {
            std::cerr << "ActionSelectionSawtoothLP::createConstraints(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
    }

    void ActionSelectionSawtoothLP::createGlobalConstraint(const std::shared_ptr<ValueFunction> &, const std::shared_ptr<State> &, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number)
    {
        con.add(IloRange(env, -IloInfinity, IloInfinity));
        con[index].setLinearCoef(var[this->getNumber(this->getVarNameWeight(0))], +1.0);
        index++;
    }

    void ActionSelectionSawtoothLP::createConstraintsKnowingInformation(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> &, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_joint_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double denominator, double difference, IloEnv &env, IloModel &model, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        // We search for the joint_history which allow us to obtain the current next_history conditionning to the next joint observation
        switch (this->current_type_of_resolution_)
        {
        case TypeOfResolution::BigM:
            this->createSawtoothBigM(vf, state, nullptr, next_hidden_state, next_joint_observation, next_joint_history, next_state, denominator, difference, env, con, var, index, t);
            break;
        case TypeOfResolution::IloIfThenResolution:
            this->createSawtoothIloIfThen(vf, state, nullptr, next_hidden_state, next_joint_observation, next_joint_history, next_state, denominator, difference, env, model, var, t);
            break;
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

    void ActionSelectionSawtoothLP::createInitialConstraints(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        auto under_pb = ActionSelectionBase::world_->getUnderlyingProblem();

        number recover = 0;
        double Qrelaxation;

        con.add(IloRange(env, -IloInfinity, 0));
        con[index].setLinearCoef(var[this->getNumber(this->getVarNameWeight(0))], +1.0);

        // Go over all actions
        for (const auto &action : *under_pb->getActionSpace(t))
        {
            for (const auto &joint_history : state->toOccupancyState()->getJointHistories())
            {
                //<! 1.c.4 get variable a(u|o) and set constant
                recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action->toAction(), joint_history));

                Qrelaxation = this->getQValueRelaxation(vf, state, joint_history, action->toAction(), t);
                con[index].setLinearCoef(var[recover], -Qrelaxation);
            }
        }
        index++;
    }

    void ActionSelectionSawtoothLP::createSawtoothBigM(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> &, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double denominator, double difference, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        try
        {
            auto under_pb = ActionSelectionBase::world_->getUnderlyingProblem();
            number recover = 0;

            con.add(IloRange(env, -IloInfinity, this->bigM_value_));
            con[index].setLinearCoef(var[this->getNumber(this->getVarNameWeight(0))], +1.0);

            // Go over all actions
            for (const auto &action : *under_pb->getActionSpace(t))
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

    void ActionSelectionSawtoothLP::createSawtoothIloIfThen(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> &, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double denominator, double difference, IloEnv &env, IloModel &model, IloNumVarArray &var, number t)
    {
        try
        {
            auto under_pb = ActionSelectionBase::world_->getUnderlyingProblem();
            number recover = 0;

            IloExpr expr(env);
            //<! 1.c.1 get variable v and set coefficient of variable v
            expr = var[this->getNumber(this->getVarNameWeight(0))];

            // Go over all actions
            for (const auto &action : *under_pb->getActionSpace(t))
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

    void ActionSelectionSawtoothLP::createObjectiveFunction(const std::shared_ptr<ValueFunction> &, const std::shared_ptr<State> &, IloNumVarArray &var, IloObjective &obj, number)
    {
        // <! 1.a get variable v
        auto recover = this->getNumber(this->getVarNameWeight(0));
        obj.setLinearCoef(var[recover], 1);
    }

    // ************************************************************************
    // ************************************************************************
    // ******************* Function for calculation ***************************
    // ************************************************************************
    // ************************************************************************

    double ActionSelectionSawtoothLP::getQValueRelaxation(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<Action> &action, number t)
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

    double ActionSelectionSawtoothLP::computeDifference(const std::shared_ptr<ValueFunction> &vf, const Pair<std::shared_ptr<State>, double> &state_AND_value, number t)
    {
        // Compute the difference v_kappa - V_kappa i.e. the value of point at t+1 minus the relaxation value of the point at t+1

        double current_upper_bound = state_AND_value.second;                                                                                                 //Get Value for the state at t+1
        double initial_upper_bound = vf->getInitFunction()->operator()(state_AND_value.first->toOccupancyState()->getOneStepUncompressedOccupancy(), t + 1); // Get relaxation value of the state at t+1

        return current_upper_bound - initial_upper_bound;
    }

    double ActionSelectionSawtoothLP::computeSawtooth(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, double denominator, double difference, number t)
    {
        double Qrelaxation = this->getQValueRelaxation(vf, state, joint_history, action, t);
        double SawtoothRatio = 0.0;
        if (joint_history->expand(next_observation) == next_joint_history)
        {
            SawtoothRatio = this->getSawtoothMinimumRatio(vf, state, joint_history, action, next_hidden_state, next_observation, denominator, t);
        }

        return Qrelaxation + SawtoothRatio * difference;
    }

    double ActionSelectionSawtoothLP::getSawtoothMinimumRatio(const std::shared_ptr<ValueFunction> &, const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, double denominator, number t)
    {
        double numerator = 0.0;

        try
        {
            // Compute the numerator for the Sawtooth Ratio, i.e. we compute the \sum_{x} s(x,o) * T(x,u,x_,z_)
            // This formulation allow us to tranform the problem at t+1, like the next_one_step_uncompressed_occupancy_state
            // therefore, we can have the sawtooth ration equivalent to 1.

            auto compressed_occupancy_state = state->toOccupancyState();
            auto under_pb = std::dynamic_pointer_cast<MPOMDPInterface>(ActionSelectionBase::world_->getUnderlyingProblem());

            // Go over all hidden state  in a belief conditionning to a joint history
            for (const auto &hidden_state : compressed_occupancy_state->getBeliefAt(joint_history)->getStates())
            {
                numerator += compressed_occupancy_state->getProbability(joint_history, hidden_state) * under_pb->getDynamics(hidden_state, action, next_hidden_state, next_observation, t);
            }
        }
        catch (const std::exception &exc)
        {
            std::cerr << "ActionSelectionSawtoothLP::getSawtoothMinimumRatio(..) exception caught: " << exc.what() << std::endl;
            exit(-1);
        }
        return numerator / denominator;
    }

    // ************************************************************************
    // ************************************************************************
    // ******************* Other Function for LP ***************************
    // ************************************************************************
    // ************************************************************************

    void ActionSelectionSawtoothLP::createDecentralizedVariables(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t)
    {
        return DecentralizedLP::createDecentralizedVariablesOccupancy(vf, state, env, var, index, t);
    }

    void ActionSelectionSawtoothLP::createDecentralizedConstraints(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        return DecentralizedLP::createDecentralizedConstraintsOccupancy(vf, state, env, con, var, index, t);
    }

    std::shared_ptr<Action> ActionSelectionSawtoothLP::getVariableResult(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const IloCplex &cplex, const IloNumVarArray &var, number t)
    {
        return DecentralizedLP::getVariableResultOccupancy(vf, state, cplex, var, t);
    }

    std::shared_ptr<Joint<std::shared_ptr<Observation>>> ActionSelectionSawtoothLP::determineNextJointObservation(const std::shared_ptr<JointHistoryInterface> &next_joint_history, number)
    {
        return std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(next_joint_history->getLastObservation());
    }

    Pair<std::shared_ptr<State>, double> ActionSelectionSawtoothLP::evaluate(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state_tmp, const std::shared_ptr<Action> &decision_rule, number t)
    {
        auto occupancy_mdp = std::static_pointer_cast<OccupancyMDP>(ActionSelectionBase::world_);
        auto occupancy_state = state_tmp->toOccupancyState();

        std::shared_ptr<State> argmin_;
        double min_ext = 0.0;

        for (const auto &joint_history : occupancy_state->getJointHistories())
        {
            auto action = occupancy_mdp->applyDecisionRule(occupancy_state, joint_history, decision_rule, t);
            min_ext += this->getQValueRelaxation(vf, occupancy_state, joint_history, action->toAction(), t);
        }

        // Go over all element in the support
        for (const auto &point_value : std::dynamic_pointer_cast<TabularValueFunction>(vf)->getRepresentation(t + 1))
        {
            auto point = point_value.first->toOccupancyState()->getOneStepUncompressedOccupancy();
            double difference = this->computeDifference(vf, point_value, t);

            double max_value = -std::numeric_limits<double>::max();

            // Go over all joint history
            for (const auto &next_joint_history : point->getJointHistories())
            {
                auto next_observation = this->determineNextJointObservation(next_joint_history, t);

                // Go over all hidden state in the belief conditionning to the joitn history
                for (const auto &next_hidden_state : point->getBeliefAt(next_joint_history)->getStates())
                {
                    double denominator = point->getProbability(next_joint_history, next_hidden_state);

                    double total = 0.0;

                    for (const auto &joint_history : occupancy_state->getJointHistories())
                    {
                        auto action = occupancy_mdp->applyDecisionRule(occupancy_state, joint_history, decision_rule, t);
                        total += this->computeSawtooth(vf, occupancy_state, action, joint_history, next_hidden_state, next_observation, next_joint_history, denominator, difference, t);
                    }

                    // determine the max for the support
                    if (max_value < total)
                    {
                        max_value = total;
                    }
                }
            }

            // determine the min for the point at t+1
            if (min_ext > max_value)
            {
                min_ext = max_value;
                argmin_ = point;
            }
        }
        return std::make_pair(argmin_, min_ext);
    }
}

#endif