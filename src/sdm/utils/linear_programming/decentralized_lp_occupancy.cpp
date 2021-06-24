#include <sdm/utils/linear_programming/decentralized_lp_occupancy.hpp>

namespace sdm
{
    DecentralizedLPOccupancy::DecentralizedLPOccupancy() {}
    DecentralizedLPOccupancy::DecentralizedLPOccupancy(const std::shared_ptr<SolvableByHSVI>& world) : DecentralizedLP(world) {}

    void DecentralizedLPOccupancy::createDecentralizedVariables(const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t)
    {
        auto under_pb = this->world_->getUnderlyingProblem();

        this->createDecentralizedVariablesJoint(state, env, var, index, t);

        for (auto agent = 0; agent < under_pb->getNumAgents(); ++agent)
        {
            this->createDecentralizedVariablesIndividual(state, env, var, index, t,agent);
        }
    }

    void DecentralizedLPOccupancy::createDecentralizedConstraints(const std::shared_ptr<State>& state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        auto under_pb = this->world_->getUnderlyingProblem();

        this->createDecentralizedConstraintsJoint(state, env, con, var, index, t);
        this->createDecentralizedControlConstraints(state, env, con, var, index, t);

        for (number agent = 0; agent < under_pb->getNumAgents(); ++agent)
        {
            this->createDecentralizedConstraintsIndividual(state, env, con, var, index, t,agent);
        }
    }

    std::shared_ptr<Action> DecentralizedLPOccupancy::getVariableResult(const std::shared_ptr<State> &state,const IloCplex &cplex, const IloNumVarArray &var, number t)
    {
        number index = 0;
        std::vector<std::vector<std::shared_ptr<Item>>> actions;
        std::vector<std::vector<std::shared_ptr<Item>>> joint_histories;

        auto under_pb = this->world_->getUnderlyingProblem();

        for (number agent = 0; agent < under_pb->getNumAgents(); agent++)
        {
            actions.push_back({});
            joint_histories.push_back({});

            auto action_and_history_individual = this->getVariableResultIndividual(state,cplex,var,t,agent);

            actions[agent].push_back(action_and_history_individual.first);
            joint_histories[agent].push_back(action_and_history_individual.second);
        }
        
        return std::make_shared<JointDeterministicDecisionRule>(joint_histories,actions);
    }
}