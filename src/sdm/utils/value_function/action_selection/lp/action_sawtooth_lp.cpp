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
            //<! 0.b Build variables v_0 = objective variable!
            std::string VarName = this->getVarNameWeight(0);
            var.add(IloNumVar(env, -IloInfinity, IloInfinity, VarName.c_str()));
            this->setNumber(VarName, index++);

            //<! Define variables \omega_k(x',o')
            // Go over all Point Set in t+1
            for (const auto &s_v : std::dynamic_pointer_cast<TabularValueFunction>(vf)->getRepresentation(t + 1))
            {
                const auto &s__ = s_v.first->toOccupancyState()->getOneStepUncompressedOccupancy();

                // Go over all Joint History Next
                for (const auto &next_jhistory : s__->getJointHistories())
                {
                    // Go over all Hidden State in the next one step uncomppresed occupancy state
                    for (const auto next_state : s__->getBeliefAt(next_jhistory)->getStates())
                    {
                        // <! \omega_k(x',o')
                        VarName = this->getVarNameWeightedStateJointHistory(s__, next_state, next_jhistory);
                        var.add(IloBoolVar(env, 0, 1, VarName.c_str()));
                        this->setNumber(VarName, index++);
                    }
                }
            }

            // Create Decentralized Variables
            DecentralizedLP::createVariables(vf, state, env, var, index, t);
    }



    //<!  Build sawtooth constraints v - \sum_{u} a(u|o) * Q(k,s,o,u,y,z, diff, t  ) + \omega_k(y,<o,z>)*M <= M,  \forall k, y,<o,z>
    //<!  Build sawtooth constraints  Q(k,s,o,u,y,z, diff, t ) = (v_k - V_k) \frac{\sum_{x} s(x,o) * p(x,u,z,y)}}{s_k(y,<o,z>)},  \forall a(u|o)
    void ActionSelectionSawtoothLP::createConstraints(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloModel &model, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        auto value_function = std::dynamic_pointer_cast<ValueFunction>(vf);
        assert(value_function->getInitFunction() != nullptr);

            auto compressed_occupancy_state = state->toOccupancyState();

            if (value_function->getSupport(t + 1).empty())
            {
                this->createInitialConstraints(value_function, state, env, con, var, index, t);
            }

            else
            {
                // Go over all points in the point set at t+1
                for (const auto &s_v : std::dynamic_pointer_cast<TabularValueFunction>(vf)->getRepresentation(t + 1))
                {
                    const auto &s__ = s_v.first->toOccupancyState()->getOneStepUncompressedOccupancy();

                    // Go over all Joint History Next
                    for (const auto &next_jhistory : s__->getJointHistories())
                    {
                        // Go over all Hidden State in the next one step uncomppresed occupancy state
                        for (const auto next_state : s__->getBeliefAt(next_jhistory)->getStates())
                        {

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
                }
                    
                this->createOmegaConstraints(env, con, var, index);
            }

    }



    void ActionSelectionSawtoothLP::createOmegaConstraints( IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index)
    {

        number recover = 0;


                 // Go over all points in the point set at t+1
                for (const auto &s_v : std::dynamic_pointer_cast<TabularValueFunction>(vf)->getRepresentation(t + 1))
                {
                    const auto &s__ = s_v.first->toOccupancyState()->getOneStepUncompressedOccupancy();

                    // Build constraint \sum{x',o'} \omega_k(x',o') = 1
                    con.add(IloRange(env, 1.0, 1.0));

                    // Go over all Joint History Next
                    for (const auto &next_jhistory : s__->getJointHistories())
                    {
                        // Go over all Hidden State in the next one step uncomppresed occupancy state
                        for (const auto next_state : s__->getBeliefAt(next_jhistory)->getStates())
                        {


                            // <! \omega_k(x',o')
                            auto VarName = this->getVarNameWeightedStateJointHistory(ostate, next_state, next_jhistory);
                            recover = this->getNumber(VarName);
                            con[index].setLinearCoef(var[recover], +1.0);
                        }
                    }
                    index++;
                }
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

                con[index].setLinearCoef(var[recover], - value_function->getQValueRelaxation(state, joint_history, action->toAction(), t));
            }
        }
        index++;
    }


    void ActionSelectionSawtoothLP::createSawtoothBigM(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> &, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double denominator, double difference, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
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
                    auto s_ = state->toOccupancyState(); double scalar = 0; 
                    for (const auto &hidden_state : compressed_occupancy_state->getBeliefAt(joint_history)->getStates()){
                       scalar += s_->getProbability(joint_history, hidden_state) * vf->getSawtoothValueAt(/* support from current point */hidden_state, joint_history, action->toAction(), next_observation, /* witness point */ next_state, /* support of witness point */  next_hidden_state, next_joint_history,  /* to be kept */ t)     
                    }

                    con[index].setLinearCoef(var[recover], -scalar);
                }
            }
            // <! \omega_k(x',o') * BigM
            recover = this->getNumber(this->getVarNameWeightedStateJointHistory(next_state, next_hidden_state, next_joint_history));
            con[index].setLinearCoef(var[recover], this->bigM_value_);

            index++;
     }


    void ActionSelectionSawtoothLP::createSawtoothIloIfThen(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> &, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double denominator, double difference, IloEnv &env, IloModel &model, IloNumVarArray &var, number t)
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
                    auto s_ = state->toOccupancyState(); double scalar = 0; 
                    for (const auto &hidden_state : compressed_occupancy_state->getBeliefAt(joint_history)->getStates()){
                       scalar += s_->getProbability(joint_history, hidden_state) * vf->getSawtoothValueAt(/* support from current point */hidden_state, joint_history, action->toAction(), next_observation, /* witness point */ next_state, /* support of witness point */  next_hidden_state, next_joint_history,  /* to be kept */ t)     
                    }

                    expr -= var[recover] * scalar;
                }
            }

            // <! get variable \omega_k(x',o')
            recover = this->getNumber(this->getVarNameWeightedStateJointHistory(next_state, next_hidden_state, next_joint_history));
            model.add(IloIfThen(env, var[recover] > 0, expr <= 0));
    }

}

#endif