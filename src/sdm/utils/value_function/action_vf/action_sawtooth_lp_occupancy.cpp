#include <sdm/utils/value_function/action_vf/action_sawtooth_lp_occupancy.hpp>

namespace sdm
{
    ActionVFSawtoothLPOccupancy::ActionVFSawtoothLPOccupancy(){}
    ActionVFSawtoothLPOccupancy::ActionVFSawtoothLPOccupancy(const std::shared_ptr<SolvableByHSVI>& world,TypeOfResolution current_type_of_resolution, number bigM_value, TypeSawtoothLinearProgram type_sawtooth_resolution) : ActionVFSawtoothLP(world,current_type_of_resolution,bigM_value,type_sawtooth_resolution){}
    

    void ActionVFSawtoothLPOccupancy::createSawtoothBigM(const std::shared_ptr<ValueFunction>&vf,const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface>& joint_history, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double probability, double difference, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        try
        {
            auto under_pb = ActionVFBase::world_->getUnderlyingProblem();

            number recover = 0;

            con.add(IloRange(env, -IloInfinity, this->bigM_value_));
            con[index].setLinearCoef(var[this->getNumber(this->getVarNameWeight(0))], +1.0);

            // Go over all actions
            for (const auto &action : *under_pb->getActionSpace(t))
            {
                //<! 1.c.4 get variable a(u|o) and set constant
                recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action->toAction(), joint_history));
                con[index].setLinearCoef(var[recover], - this->getQValueRealistic(vf,state, joint_history, action->toAction(), next_hidden_state, next_observation, probability, difference,t));
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

    void ActionVFSawtoothLPOccupancy::createSawtoothIloIfThen(const std::shared_ptr<ValueFunction>&vf,const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface>& joint_history, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double probability, double difference, IloEnv &env, IloModel &model, IloNumVarArray &var,number t)
    {
        try
        {
            number recover = 0;

            auto under_pb = ActionVFBase::world_->getUnderlyingProblem();

            IloExpr expr(env);
            //<! 1.c.1 get variable v and set coefficient of variable v
            expr = var[this->getNumber(this->getVarNameWeight(0))];

            // Go over all actions
            for(const auto & action : *under_pb->getActionSpace(t))
            { 
                recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action->toAction(), joint_history));
                //<! 1.c.4 get variable a(u|o) and set constant 
                expr -= this->getQValueRealistic(vf,state, joint_history, action->toAction(), next_hidden_state, next_observation, probability, difference,t) * var[recover];
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

    void ActionVFSawtoothLPOccupancy::createInitialConstrainte(const std::shared_ptr<ValueFunction>&,const std::shared_ptr<State> &, IloEnv &, IloRangeArray &, IloNumVarArray &, number &, number ){}

}