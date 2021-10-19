#ifdef WITH_CPLEX
#include <sdm/utils/value_function/action_selection/lp/action_sawtooth_lp_serial.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>
#include <sdm/core/state/interface/serial_interface.hpp>

#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/world/serial_mmdp.hpp>
#include <sdm/world/serial_mpomdp.hpp>
#include <sdm/world/base/mpomdp_interface.hpp>

namespace sdm
{
    ActionSelectionSawtoothLPSerial::ActionSelectionSawtoothLPSerial() {}
    ActionSelectionSawtoothLPSerial::ActionSelectionSawtoothLPSerial(const std::shared_ptr<SolvableByHSVI> &world, TypeOfResolution current_type_of_resolution,
                                                       number bigM_value, TypeSawtoothLinearProgram type_of_linear_programm)
        : ActionSelectionSawtoothLP(world, current_type_of_resolution, bigM_value, type_of_linear_programm)
    {
    }

    std::shared_ptr<Joint<std::shared_ptr<Observation>>> ActionSelectionSawtoothLPSerial::determineNextJointObservation(const std::shared_ptr<JointHistoryInterface> &next_joint_history, number t)
    {
        auto under_pb = std::dynamic_pointer_cast<SerialMPOMDPInterface>(ActionSelectionBase::world_->getUnderlyingProblem());

        // Check if last agent
        bool last_agent = std::dynamic_pointer_cast<SerialProblemInterface>(ActionSelectionBase::world_)->isLastAgent(t);

        // Get next observation
        auto next_joint_observation = (last_agent) ? std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(next_joint_history->getLastObservation()) : under_pb->getDefaultObservation();

        return next_joint_observation;
    }

    void ActionSelectionSawtoothLPSerial::createSawtoothBigM(const std::shared_ptr<ValueFunction> &, const std::shared_ptr<State> &, const std::shared_ptr<JointHistoryInterface> &, const std::shared_ptr<State> &, const std::shared_ptr<Observation> &, const std::shared_ptr<JointHistoryInterface> &, const std::shared_ptr<State> &, double, double, IloEnv &, IloRangeArray &, IloNumVarArray &, number &, number)
    {
        throw sdm::exception::NotImplementedException();
        // try
        // {
        //     auto under_pb = std::dynamic_pointer_cast<SerialMMDP>(this->world_->getUnderlyingProblem());

        //     auto agent_id = under_pb->getAgentId(t);
        //     // Gets the current individual history conditional on the current joint history
        //     auto indiv_history = joint_history->getIndividualHistory(agent_id);

        //     number recover = 0;

        //     con.add(IloRange(env, -IloInfinity, this->bigM_value_));
        //     con[index].setLinearCoef(var[this->getNumber(this->getVarNameWeight(0))], +1.0);

        //     // Go over all actions
        //     for (const auto &action : *under_pb->getActionSpace(t))
        //     {
        //         //<! 1.c.4 get variable a(u|o) and set constant
        //         recover = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(action->toAction(), indiv_history, agent_id))
        //         con[index].setLinearCoef(var[recover], - this->getQValueRealistic(state, joint_history, action->toAction(), next_hidden_state, next_observation, probability, difference));
        //     }

        //     // <! \omega_k(x',o') * BigM
        //     recover = this->getNumber(this->getVarNameWeightedStateJointHistory(next_state, next_hidden_state, next_joint_history));
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

    void ActionSelectionSawtoothLPSerial::createSawtoothIloIfThen(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> &, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double denominator, double difference, IloEnv &env, IloModel &model, IloNumVarArray &var, number t)
    {

        try
        {
            auto under_pb = std::dynamic_pointer_cast<SerialMMDP>(ActionSelectionBase::world_->getUnderlyingProblem());
            auto compressed_occupancy_state = state->toOccupancyState();

            // Gets the current individual history conditional on the current joint history
            number agent_id = under_pb->getAgentId(t);

            number recover = 0;
            IloExpr expr(env);
            //<! 1.c.1 get variable v and set coefficient of variable v
            expr = var[this->getNumber(this->getVarNameWeight(0))];

            // Go over all actions
            for (const auto &action : *under_pb->getActionSpace(t))
            {
                for (const auto &indiv_history : compressed_occupancy_state->getIndividualHistories(agent_id))
                {
                    recover = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(action->toAction(), indiv_history, agent_id));

                    double compute_sawtooth = 0.0;
                    for (const auto &joint_history : std::dynamic_pointer_cast<OccupancyState>(compressed_occupancy_state)->getPrivateOccupancyState(agent_id, indiv_history)->getJointHistories())
                    {
                        compute_sawtooth += this->computeSawtooth(vf, state, action->toAction(), joint_history, next_hidden_state, next_observation, next_joint_history, denominator, difference, t);
                    }
                    //<! 1.c.4 get variable a(u|o) and set constant
                    expr -= compressed_occupancy_state->getProbabilityOverIndividualHistories(agent_id, indiv_history) * compute_sawtooth * var[recover];
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

    void ActionSelectionSawtoothLPSerial::createInitialConstraints(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        auto under_pb = std::dynamic_pointer_cast<SerialMMDP>(ActionSelectionBase::world_->getUnderlyingProblem());
        number agent_id = under_pb->getAgentId(t);

        auto compressed_occupancy_state = state->toOccupancyState();

        number recover = 0;
        double Qrelaxation;

        con.add(IloRange(env, -IloInfinity, 0));
        con[index].setLinearCoef(var[this->getNumber(this->getVarNameWeight(0))], +1.0);

        // Go over all actions
        for (const auto &action : *under_pb->getActionSpace(t))
        {
            for (const auto &indiv_history : compressed_occupancy_state->getIndividualHistories(agent_id))
            {
                //<! 1.c.4 get variable a(u|o) and set constant
                recover = this->getNumber(this->getVarNameIndividualHistoryDecisionRule(action->toAction(), indiv_history, agent_id));

                Qrelaxation = 0.0;
                for (const auto &joint_history : std::dynamic_pointer_cast<OccupancyState>(compressed_occupancy_state)->getPrivateOccupancyState(agent_id, indiv_history)->getJointHistories())
                {
                    Qrelaxation += this->getQValueRelaxation(vf, compressed_occupancy_state, joint_history, action->toAction(), t);
                }

                con[index].setLinearCoef(var[recover], -Qrelaxation * compressed_occupancy_state->getProbabilityOverIndividualHistories(agent_id, indiv_history));
            }
        }
        index++;
    }

    void ActionSelectionSawtoothLPSerial::createDecentralizedVariables(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t)
    {
        //Determine the Agent
        auto under_pb = std::dynamic_pointer_cast<SerialMMDP>(ActionSelectionBase::world_->getUnderlyingProblem());
        number agent_id = under_pb->getAgentId(t);

        //Create Individual Decentralized Variable
        this->createDecentralizedVariablesIndividual(vf, state, env, var, index, t, agent_id);
    }

    void ActionSelectionSawtoothLPSerial::createDecentralizedConstraints(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        //Determine the Agent
        auto under_pb = std::dynamic_pointer_cast<SerialMMDP>(ActionSelectionBase::world_->getUnderlyingProblem());
        number agent_id = under_pb->getAgentId(t);

        //Create Individual Decentralized Constraints
        this->createDecentralizedConstraintsIndividual(vf, state, env, con, var, index, t, agent_id);
    }

    std::shared_ptr<Action> ActionSelectionSawtoothLPSerial::getVariableResult(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const IloCplex &cplex, const IloNumVarArray &var, number t)
    {
        //Determine the Agent
        auto under_pb = std::dynamic_pointer_cast<SerialMMDP>(ActionSelectionBase::world_->getUnderlyingProblem());
        number agent_id = under_pb->getAgentId(t);

        //Determine the element useful for create a DeterminiticDecisionRule
        auto action_and_history_individual = this->getVariableResultIndividual(vf, state, cplex, var, t, agent_id);

        //Create the DeterminiticDecisionRule
        return std::make_shared<DeterministicDecisionRule>(action_and_history_individual.second, action_and_history_individual.first);
    }
}

#endif