#include <sdm/utils/value_function/action_vf/action_maxplan_lp.hpp>

#include <sdm/core/state/interface/belief_interface.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>

#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/world/base/mpomdp_interface.hpp>

namespace sdm
{
        
    ActionVFMaxplanLP::ActionVFMaxplanLP() {}
    ActionVFMaxplanLP::ActionVFMaxplanLP(const std::shared_ptr<SolvableByHSVI>& world) : ActionVFBase(world), DecentralizedLP(world) {}
    
    std::shared_ptr<Action> ActionVFMaxplanLP::selectBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t)
    {
        std::shared_ptr<Action> max_decision_rule;
        double max = -std::numeric_limits<double>::max();

        // Go over all hyperplan in the Support
        for (const auto &hyperplan : vf->getSupport(t + 1))
        {
            this->tmp_representation = hyperplan->toBelief();
            auto pair_action_value = this->createLP(vf,state, t);

            // Select the Best Action 
            if (pair_action_value.second > max)
            {
                max_decision_rule = pair_action_value.first;
                max = pair_action_value.second;
            }
        }
        return max_decision_rule;
    }

    void ActionVFMaxplanLP::createVariables(const std::shared_ptr<ValueFunction>&vf, const std::shared_ptr<State> &occupancy_state, IloEnv &env, IloNumVarArray &var,number &index, number t)
    {
        //<! tracking variables
        std::string VarName;

        // re-init the set of variable names
        this->variables.clear();

        //<! Build variables v_size = objective variable!
        VarName = this->getVarNameWeight(0);
        var.add(IloNumVar(env, -IloInfinity, +IloInfinity, VarName.c_str()));
        this->setNumber(VarName, index++);

        //<! set decentralized decision rule variables
        this->createDecentralizedVariables(vf,occupancy_state, env, var, index, t);
    }

    void ActionVFMaxplanLP::createObjectiveFunction(const std::shared_ptr<ValueFunction>&, const std::shared_ptr<State> &state, IloNumVarArray &var, IloObjective &obj, number t)
    {
        auto under_pb = std::dynamic_pointer_cast<MPOMDPInterface>(ActionVFBase::world_->getUnderlyingProblem());
        auto occupancy_state = state->toOccupancyState();

        number recover = 0;
        double weight = 0.0, factor = 0.0;

        // Go over all joint history 
        for (const auto &joint_history : occupancy_state->getJointHistories())
        {
            // Go over all action
            for (const auto &action : *under_pb->getActionSpace(t))
            {
                // Compute \sum_{x} s(o,x)* discount * [ r(x,o) + \sum_{x_,z_} p(x,u,x_,z_,) * next_hyperplan(<o,z_>,x_)]
                weight = 0.0;
                
                // Go over all hidden state in the belief conditionning to a joint history
                for(const auto &hidden_state : occupancy_state->getBeliefAt(joint_history)->getStates())
                {
                    // Determine the reward for the hidden state and the action
                    factor = under_pb->getReward(hidden_state,action->toAction(),t);

                    // Go over all reachable next hidden state 
                    for (const auto &next_hidden_state : under_pb->getReachableStates(hidden_state,action->toAction(),t))
                    {
                        // Go over all reachable observation 
                        for(const auto &observation : under_pb->getReachableObservations(hidden_state,action->toAction(),next_hidden_state,t))
                        {
                            // Determine the next joint history conditionning to the observation
                            auto next_joint_history = joint_history->expand(observation->toObservation())->toJointHistory();

                            factor += this->tmp_representation->toOccupancyState()->getProbability(next_joint_history,next_hidden_state) * under_pb->getDynamics(hidden_state,action->toAction(),next_hidden_state,observation,t);
                        }
                    }
                    weight +=  occupancy_state->getProbability(joint_history,hidden_state) * ActionVFBase::world_->getDiscount(t) * factor ;
                }

                //<! 1.b get variable a(u|o)
                recover = this->getNumber(this->getVarNameJointHistoryDecisionRule(action->toAction(), joint_history));

                //<! 1.c set coefficient of variable a(u|o) i.e., s(x,o)  [ r(x,u) + \gamma \sum_{x_,z_} P(x_,z_|x,u) * \hyperplan_i(x_,o_)  ]
                obj.setLinearCoef(var[recover], weight);
            } // for all u
        }   // for all o
    }
    
    void ActionVFMaxplanLP::createConstraints(const std::shared_ptr<ValueFunction>&vf, const std::shared_ptr<State>&state, IloEnv &env, IloModel &, IloRangeArray &con, IloNumVarArray &var, number &index, number t)
    {
        this->createDecentralizedConstraints(vf,state, env, con, var, index, t);
    }
}
