#include <sdm/utils/value_function/backup/maxplan_serial_backup.hpp>
#include <sdm/world/serialized_mpomdp.hpp>

#include <sdm/core/action/det_decision_rule.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>

namespace sdm
{

    MaxPlanSerialBackup::MaxPlanSerialBackup() {}

    MaxPlanSerialBackup::MaxPlanSerialBackup(const std::shared_ptr<SolvableByHSVI> &world) : MaxPlanBackup(world)
    {}

    Pair<std::shared_ptr<State>,std::shared_ptr<Action>> MaxPlanSerialBackup::getBestActionAndMaxHyperplan(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t)
    {
        double max = -std::numeric_limits<double>::max(), value; 

        //Determine the determinitic decision rule
        auto action = this->getBestAction(vf,state,t);

        std::shared_ptr<State> max_next_step_hyperplan;

        // Go other the hyperplanes of decision step t+1
        for(const auto &hyperplan : vf->getSupport(t+1))
        {
            // Determine the max next hyperplan
            if(max < (value = this->getMaxPlanValueAt(state->toSerialOccupancyState(), action, hyperplan->toSerialOccupancyState(), t)))
            {
                max = value;
                max_next_step_hyperplan = hyperplan;
            }
        }
    }

    std::shared_ptr<State> MaxPlanSerialBackup::backup(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t)
    {
        auto pair_hyperplan_action = this->getBestActionAndMaxHyperplan(vf,state,t);
        return this->setHyperplan(vf,state, pair_hyperplan_action.first->toBelief(),pair_hyperplan_action.second, t);
    }

    std::shared_ptr<Action> MaxPlanSerialBackup::getBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t)
    {
        std::shared_ptr<Action> decision_max;
        auto serial_occupancy_state = state->toSerialOccupancyState();

        auto under_pb = std::dynamic_pointer_cast<SerializedMPOMDP>(this->world_->getUnderlyingProblem());
        auto agent = under_pb->getAgentId(t);

        // Definie local Variable
        double argmax_global = -std::numeric_limits<double>::max();  
        double decision_rule_value;

        // // Go other the hyperplanes of decision step t+1
        for(const auto &hyperplan : vf->getSupport(t+1))
        {
            decision_rule_value = 0;  
            std::shared_ptr<DecisionRule> decision_ = std::make_shared<DeterministicDecisionRule>();

            // Go over all Individual Histories in Serial Occupancy State
            for (const auto &private_history : serial_occupancy_state->getIndividualHistories(agent))
            {
                double argmax_local = -std::numeric_limits<double>::max(); 

                std::shared_ptr<SerialOccupancyInterface> private_serial_occupancy_state = serial_occupancy_state->getPrivateOccupancyState(agent,private_history)->toSerialOccupancyState();
                
                // Go over all action possible for the current agent
                for (const auto &private_action : *under_pb->getActionSpace(agent))
                {
                    double action_value = this->getMaxPlanValueAt(private_serial_occupancy_state, private_action->toAction(),hyperplan->toSerialOccupancyState(), t);
                    
                    // Take the best deterministic decision rule "decision_" for a precise hyperplan
                    if(argmax_local < action_value)
                    {
                        argmax_local = action_value;
                        decision_->setProbability(private_history,private_action->toAction(),1);
                    }
                }
                decision_rule_value += serial_occupancy_state->getProbabilityOverIndividualHistories(agent,private_history) * argmax_local;
            }
            // Take the best deterministic decision rule
            if(argmax_global < decision_rule_value)
            {
                argmax_global = decision_rule_value;
                decision_max = decision_;
            }
        }
        return decision_max;
    }

    double MaxPlanSerialBackup::getMaxPlanValueAt(const std::shared_ptr<SerialOccupancyInterface> &serial_occupancy_state, const std::shared_ptr<Action>& action, const std::shared_ptr<SerialOccupancyInterface>& next_step_hyperplan, number t)
    {
        double value = 0;
        auto under_pb = std::dynamic_pointer_cast<SerializedMPOMDP>(this->world_->getUnderlyingProblem());
        number agent = under_pb->getAgentId(t);

        // Go over all compressed serial occupancy state 
        for (const auto &compressed_joint_history : serial_occupancy_state->getJointHistories())
        {
            std::shared_ptr<Action> serial_action;

            // Determine the serial action for each different situation
            switch(action->getTypeAction())
            {
            // If it's a occupancy state, we need the JOint histories
            case TypeAction::DECISION_RULE :
                serial_action = action->toDecisionRule()->act(compressed_joint_history->getIndividualHistory(agent));
                break;
            default:
                serial_action = action;
                break;
            }

            for(const auto &compressed_serial_occupancy_state : serial_occupancy_state->getStatesAt(compressed_joint_history))
            {
                auto probability = serial_occupancy_state->getProbability(serial_occupancy_state->HiddenStateAndJointHistoryToState(compressed_serial_occupancy_state,compressed_joint_history));

                // Determine the reward 
                double immediate_reward = under_pb->getReward(compressed_serial_occupancy_state, serial_action,t);

                // Compute the next value
                double next_value = 0;

                // Go over all Reachable Serial State
                for(const auto &serialized_underlying_next_state: under_pb->getReachableStates(compressed_serial_occupancy_state, serial_action,t))
                {
                    // Go over all Reachable Observation
                    for(const auto &serial_observation :under_pb->getReachableObservations(compressed_serial_occupancy_state, serial_action, serialized_underlying_next_state,t))
                    {
                        auto joint_serial_observation = std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(serial_observation);
                        
                        auto history_next = compressed_joint_history->expand(joint_serial_observation)->toJointHistoryTree();
                        next_value += next_step_hyperplan->getProbability(next_step_hyperplan->HiddenStateAndJointHistoryToState(serialized_underlying_next_state, history_next)) * under_pb->getDynamics(compressed_serial_occupancy_state,serial_action,serialized_underlying_next_state,serial_observation,t);
                    }
                }
                value += probability * (immediate_reward + under_pb->getDiscount(t) * next_value);

            }
        }
        return value;
    }
} // namespace sdm