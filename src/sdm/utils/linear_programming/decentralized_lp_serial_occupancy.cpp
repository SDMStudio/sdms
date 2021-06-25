#include <sdm/utils/linear_programming/decentralized_lp_serial_occupancy.hpp>

namespace sdm
{
    DecentralizedLPSerialOccupancy::DecentralizedLPSerialOccupancy() {}
    DecentralizedLPSerialOccupancy::DecentralizedLPSerialOccupancy(const std::shared_ptr<SolvableByHSVI>& world) : DecentralizedLP(world) {}

    void DecentralizedLPSerialOccupancy::createDecentralizedVariables(const std::shared_ptr<ValueFunction>&, const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t)
    {
        auto under_pb = std::dynamic_pointer_cast<SerializedMPOMDP>(this->world_->getUnderlyingProblem());
        number agent_id = under_pb->getAgentId(t);

        this->createDecentralizedVariablesIndividual(state, env, var, index, t,agent_id);
    }

    void DecentralizedLPSerialOccupancy::createDecentralizedConstraints(const std::shared_ptr<ValueFunction>&, const std::shared_ptr<State>& state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        auto under_pb = std::dynamic_pointer_cast<SerializedMPOMDP>(this->world_->getUnderlyingProblem());
        number agent_id = under_pb->getAgentId(t);

        this->createDecentralizedConstraintsIndividual(state, env, con, var, index, t,agent_id);
    }

    std::shared_ptr<Action> DecentralizedLPSerialOccupancy::getVariableResult(const std::shared_ptr<ValueFunction>&, const std::shared_ptr<State> &state,const IloCplex &cplex, const IloNumVarArray &var, number t)
    {
        auto under_pb = std::dynamic_pointer_cast<SerializedMPOMDP>(this->world_->getUnderlyingProblem());
        number agent_id = under_pb->getAgentId(t);

        auto action_and_history_individual = this->getVariableResultIndividual(state,cplex,var,t,agent_id);
        
        return std::make_shared<DeterministicDecisionRule>(action_and_history_individual.second,action_and_history_individual.first);
    }
}