#ifdef WITH_CPLEX
#include <sdm/utils/value_function/action_selection/lp/action_sawtooth_lp_serial.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>
#include <sdm/core/state/interface/serial_interface.hpp>

#include <sdm/world/serial_mpomdp.hpp>
#include <sdm/world/base/mpomdp_interface.hpp>
#include <sdm/world/serial_occupancy_mdp.hpp>

namespace sdm
{
    ActionSelectionSawtoothLPSerial::ActionSelectionSawtoothLPSerial() {}

    ActionSelectionSawtoothLPSerial::ActionSelectionSawtoothLPSerial(const std::shared_ptr<SolvableByDP> &world, Config config) : ActionSelectionSawtoothLP(world, config)
    {
        if (auto derived = std::dynamic_pointer_cast<SerialOccupancyMDP>(world))
            this->serial_oMDP = derived;
        else
            throw sdm::exception::TypeError("Action sawtooth lp serial is only available for worlds inheriting from 'SerialProblemInterface'.");
    }

    ActionSelectionSawtoothLPSerial::ActionSelectionSawtoothLPSerial(const std::shared_ptr<SolvableByDP> &world, TypeOfResolution current_type_of_resolution,
                                                                     number bigM_value, TypeSawtoothLinearProgram type_of_linear_programm)
        : ActionSelectionSawtoothLP(world, current_type_of_resolution, bigM_value, type_of_linear_programm)
    {
        if (auto derived = std::dynamic_pointer_cast<SerialOccupancyMDP>(world))
            this->serial_oMDP = derived;
        else
            throw sdm::exception::TypeError("Action sawtooth lp serial is only available for worlds inheriting from 'SerialProblemInterface'.");
    }

    std::shared_ptr<SerialOccupancyMDP> ActionSelectionSawtoothLPSerial::getSerialOccupancyMDP() const
    {
        return this->serial_oMDP;
    }

    void ActionSelectionSawtoothLPSerial::createVariables(const std::shared_ptr<ValueFunctionInterface> &, const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t)
    {
        //<! 0.b Build variables v_0 = objective variable!
        std::string VarName = this->getVarNameWeight(0);
        var.add(IloNumVar(env, -IloInfinity, IloInfinity, VarName.c_str()));
        this->setNumber(VarName, index++);

        //<! Define variables \omega_k(x',o')
        // Go over all Point Set in t+1
        for (const auto &point_k : getSawtoothValueFunction()->getRepresentation(t + 1))
        {
            const auto &s_k = point_k.first->toOccupancyState();

            // Go over all Joint History Next
            for (const auto &next_jhistory : s_k->getJointHistories())
            {
                // Go over all Hidden State in the next one step uncomppresed occupancy state
                for (const auto next_state : s_k->getBeliefAt(next_jhistory)->getStates())
                {
                    // <! \omega_k(x',o')
                    VarName = this->getVarNameWeightedStateJointHistory(s_k, next_state, next_jhistory);
                    var.add(IloBoolVar(env, 0, 1, VarName.c_str()));
                    this->setNumber(VarName, index++);
                }
            }
        }
        // Create Individual Variables
        IndividualLP::createVariables(getSawtoothValueFunction(), state, env, var, index, t, this->getSerialOccupancyMDP()->getAgentId(t));
    }

    void ActionSelectionSawtoothLPSerial::createConstraints(const std::shared_ptr<ValueFunctionInterface> &, const std::shared_ptr<State> &occupancy_state, IloEnv &env, IloModel &model, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        assert(getSawtoothValueFunction()->getInitFunction() != nullptr);

        auto compressed_occupancy_state = occupancy_state->toOccupancyState();

        if (getSawtoothValueFunction()->getSupport(t + 1).empty())
        {
            this->createInitialConstraints(compressed_occupancy_state, env, con, var, index, t);
        }
        else
        {
            // Go over all points in the point set at t+1
            for (const auto &point_k : getSawtoothValueFunction()->getRepresentation(t + 1))
            {
                const auto &s_k = point_k.first->toOccupancyState();

                // Go over all Joint History Next
                for (const auto &next_jhistory : s_k->getJointHistories())
                {
                    // Determine the next Joint observation thanks to the next joint history
                    auto next_joint_observation = this->determineNextJointObservation(next_jhistory, t);

                    // Go over all Hidden State in the next one step uncomppresed occupancy state
                    for (const auto &next_state : s_k->getBeliefAt(next_jhistory)->getStates())
                    {
                        // We search for the joint_history which allow us to obtain the current next_history conditionning to the next joint observation
                        switch (this->current_type_of_resolution_)
                        {
                        case TypeOfResolution::BigM:
                            this->createSawtoothBigM(compressed_occupancy_state, s_k, next_state, next_jhistory, next_joint_observation, env, con, var, index, t);
                            break;
                        case TypeOfResolution::IloIfThenResolution:
                            this->createSawtoothIloIfThen(compressed_occupancy_state, s_k, next_state, next_jhistory, next_joint_observation, env, model, var, t);
                            break;
                        }
                    }
                }
            }

            this->createOmegaConstraints(env, con, var, index, t);
        }
        // Create Individual Constraints
        IndividualLP::createConstraints(getSawtoothValueFunction(), occupancy_state, env, model, con, var, index, t, this->getSerialOccupancyMDP()->getAgentId(t));
    }

    std::shared_ptr<Action> ActionSelectionSawtoothLPSerial::getVariableResult(const std::shared_ptr<ValueFunctionInterface> &, const std::shared_ptr<State> &state, const IloCplex &cplex, const IloNumVarArray &var, number t)
    {
        // Determine the element useful for create a DeterminiticDecisionRule
        return IndividualLP::getVariableResult(getSawtoothValueFunction(), state, cplex, var, t, this->getSerialOccupancyMDP()->getAgentId(t));
    }

    void ActionSelectionSawtoothLPSerial::createInitialConstraints(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        auto serial_mpomdp = this->getSerialOccupancyMDP()->getUnderlyingSerialMPOMDP();
        auto relaxation = std::static_pointer_cast<RelaxedValueFunction>(getSawtoothValueFunction()->getInitFunction());
        number recover = 0;

        double Qrelaxation;
        number agent_id = this->getSerialOccupancyMDP()->getAgentId(t);
        auto compressed_occupancy_state = std::dynamic_pointer_cast<OccupancyState>(occupancy_state);

        con.add(IloRange(env, -IloInfinity, 0));
        con[index].setLinearCoef(var[this->getNumber(this->getVarNameWeight(0))], +1.0);

        // Go over all actions
        auto action_end_iter = serial_mpomdp->getActionSpace(t)->end();
        for (auto action_iter = serial_mpomdp->getActionSpace(t)->begin(); !action_iter->equal(action_end_iter); action_iter = action_iter->next())
        {
            auto action = action_iter->getCurrent();
            for (const auto &indiv_history : compressed_occupancy_state->getIndividualHistories(agent_id))
            {
                //<! 1.c.4 get variable a(u|o) and set constant
                recover = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(action, indiv_history, agent_id));

                Qrelaxation = 0.0;
                for (const auto &joint_history : compressed_occupancy_state->getPrivateOccupancyState(agent_id, indiv_history)->getJointHistories())
                {
                    Qrelaxation += compressed_occupancy_state->getProbability(joint_history) * relaxation->getQValueAt(compressed_occupancy_state->getBeliefAt(joint_history), action, t);
                }
                con[index].setLinearCoef(var[recover], -Qrelaxation);
            }
        }
        index++;
    }

    // void ActionSelectionSawtoothLPSerial::createSawtoothBigM(const std::shared_ptr<ValueFunctionInterface> &, const std::shared_ptr<State> &, const std::shared_ptr<JointHistoryInterface> &, const std::shared_ptr<State> &, const std::shared_ptr<Observation> &, const std::shared_ptr<JointHistoryInterface> &, const std::shared_ptr<State> &, double, double, IloEnv &, IloRangeArray &, IloNumVarArray &, number &, number)
    void ActionSelectionSawtoothLPSerial::createSawtoothBigM(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<OccupancyStateInterface> &s_k, const std::shared_ptr<State> &next_state, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<Observation> &next_observation, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        throw sdm::exception::NotImplementedException("NotImplementedException raised in ActionSelectionSawtoothLPSerial::createSawtoothBigM");
        // try
        // {
        //     auto underlying_problem = std::dynamic_pointer_cast<SerialMMDP>(this->world_->getUnderlyingProblem());

        //     auto agent_id = underlying_problem->getAgentId(t);
        //     // Gets the current individual history conditional on the current joint history
        //     auto indiv_history = joint_history->getIndividualHistory(agent_id);

        //     number recover = 0;

        //     con.add(IloRange(env, -IloInfinity, this->bigM_value_));
        //     con[index].setLinearCoef(var[this->getNumber(this->getVarNameWeight(0))], +1.0);

        //     // Go over all actions
        //     for (const auto &action : *underlying_problem->getActionSpace(t))
        //     {
        //         //<! 1.c.4 get variable a(u|o) and set constant
        //         recover = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(action, indiv_history, agent_id))
        //         con[index].setLinearCoef(var[recover], - this->getQValueRealistic(state, joint_history, action, next_state, next_observation, probability, difference));
        //     }

        //     // <! \omega_k(x',o') * BigM
        //     recover = this->getNumber(this->getVarNameWeightedStateJointHistory(next_state, next_state, next_joint_history));
        //     con[index].setLinearCoef(var[recover], this->bigM_value_);

        //     index++;
        // }
        // catch (const std::exception &exc)
        // {
        //     // catch anything thrown within try block that derives from std::exception
        //     std::cerr << "SawtoothValueFunctionLP<TState, TAction, TValue>::setGreedySawtoothBigM(..) exception caught: " << exc.what() << std::endl;
        //     exit(-1);
        // }
    }

    void ActionSelectionSawtoothLPSerial::createSawtoothIloIfThen(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<OccupancyStateInterface> &s_k, const std::shared_ptr<State> &next_state, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<Observation> &next_observation, IloEnv &env, IloModel &model, IloNumVarArray &var, number t)
    {
        auto serial_mpomdp = this->getSerialOccupancyMDP()->getUnderlyingSerialMPOMDP();
        auto relaxation = std::static_pointer_cast<RelaxedValueFunction>(sawtooth_vf->getInitFunction());
        number recover = 0;
        double coef, ratio, difference = this->sawtooth_vf->getValueAt(s_k, t + 1) - this->sawtooth_vf->getRelaxedValueAt(s_k, t + 1);

        // Gets the current individual history conditional on the current joint history
        number agent_id = this->getSerialOccupancyMDP()->getAgentId(t);
        auto compressed_occupancy_state = std::dynamic_pointer_cast<OccupancyState>(occupancy_state);

        IloExpr expr(env);
        //<! 1.c.1 get variable v and set coefficient of variable v
        expr = var[this->getNumber(this->getVarNameWeight(0))];

        // Go over all actions

        auto action_end_iter = serial_mpomdp->getActionSpace(t)->end();
        for (auto action_iter = serial_mpomdp->getActionSpace(t)->begin(); !action_iter->equal(action_end_iter); action_iter = action_iter->next())
        {
            auto action = action_iter->getCurrent();
            for (const auto &indiv_history : compressed_occupancy_state->getIndividualHistories(agent_id))
            {
                recover = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(action, indiv_history, agent_id));

                coef = 0.0;
                for (const auto &joint_history : compressed_occupancy_state->getPrivateOccupancyState(agent_id, indiv_history)->getJointHistories())
                {
                    auto belief = compressed_occupancy_state->getBeliefAt(joint_history);
                    ratio = this->computeRatio(this->getSerialOccupancyMDP(), belief, joint_history, action, /* witness point */ s_k, /* support of witness point */ next_state, next_joint_history, /* to be kept */ next_observation, t);
                    coef += compressed_occupancy_state->getProbability(joint_history) * (relaxation->getQValueAt(belief, action, t) + difference * ratio);
                }
                //<! 1.c.4 get variable a(u|o) and set constant
                expr -= var[recover] * coef;
            }
        }

        // <! get variable \omega_k(x',o')
        recover = this->getNumber(this->getVarNameWeightedStateJointHistory(s_k, next_state, next_joint_history));
        model.add(IloIfThen(env, var[recover] > 0, expr <= 0));
    }

    std::shared_ptr<JointObservation> ActionSelectionSawtoothLPSerial::determineNextJointObservation(const std::shared_ptr<JointHistoryInterface> &next_joint_history, number t)
    {
        return (this->getSerialOccupancyMDP()->isLastAgent(t)) ? std::static_pointer_cast<JointObservation>(next_joint_history->getLastObservation()) : this->getSerialOccupancyMDP()->getDefaultObservation();
    }
}

#endif