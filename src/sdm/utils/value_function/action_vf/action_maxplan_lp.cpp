#include <sdm/utils/value_function/action_vf/action_maxplan_lp.hpp>
#include <sdm/core/state/interface/belief_interface.hpp>
#include <sdm/world/base/mpomdp_interface.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
namespace sdm
{
        
    ActionVFMaxplanLP::ActionVFMaxplanLP() {}
    ActionVFMaxplanLP::ActionVFMaxplanLP(const std::shared_ptr<SolvableByHSVI>& world) : ActionVFBase<std::shared_ptr<State>>(world), DecentralizedLP(world) {}
    
    Pair<std::shared_ptr<Action>,std::shared_ptr<State>> ActionVFMaxplanLP::selectBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t)
    {
        std::shared_ptr<State> next_hyperplan;
        std::shared_ptr<Action> max_decision_rule;
        double max = -std::numeric_limits<double>::max();

        for (const auto &hyperplan : vf->getSupport(t + 1))
        {
            this->tmp_representation = hyperplan->toBelief()->getVectorInferface();
            auto pair_action_value = this->createLP(state, t);

            if (pair_action_value.second > max)
            {
                max_decision_rule = pair_action_value.first;
                next_hyperplan = hyperplan;
                max = pair_action_value.second;
            }
        }
        return std::make_pair(max_decision_rule,next_hyperplan);
    }

    void ActionVFMaxplanLP::createVariables(const std::shared_ptr<State> &occupancy_state, IloEnv &env, IloNumVarArray &var, number t)
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
        this->createDecentralizedVariables(occupancy_state, env, var, index, t);
    }

    void ActionVFMaxplanLP::createObjectiveFunction(const std::shared_ptr<State> &state, IloNumVarArray &var, IloObjective &obj, number t)
    {
        auto under_pb = std::dynamic_pointer_cast<MPOMDPInterface>(ActionVFBase::world_->getUnderlyingProblem());
        auto occupancy_state = state->toOccupancyState();

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

                                auto joint_history_next = joint_history->expand(joint_observation)->toJointHistory();
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
    
    void ActionVFMaxplanLP::createConstraints(const std::shared_ptr<State>& occupancy_state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t) {}
}
