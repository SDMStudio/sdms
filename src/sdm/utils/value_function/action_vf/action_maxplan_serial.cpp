#include <sdm/utils/value_function/action_vf/action_maxplan_serial.hpp>
#include <sdm/core/action/det_decision_rule.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/world/serialized_mpomdp.hpp>

namespace sdm
{
    ActionVFMaxplanSerial::ActionVFMaxplanSerial() {}

    ActionVFMaxplanSerial::ActionVFMaxplanSerial(const std::shared_ptr<SolvableByHSVI> &world) : ActionVFBase(world) {}

    std::shared_ptr<Action> ActionVFMaxplanSerial::selectBestAction(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t)
    {
        // Definie local Variable
        double argmax_global = -std::numeric_limits<double>::max();
        std::shared_ptr<Action> decision_max;
        std::shared_ptr<State> best_hyperplan;

        // // Go other the hyperplanes of decision step t+1
        for (const auto &hyperplan : vf->getSupport(t + 1))
        {
            auto decision_and_value = this->selectBestActionKnowingNextHyperplan(vf, state, hyperplan, t);

            // Take the best deterministic decision rule and best hyperplan associated
            if (argmax_global < decision_and_value.second)
            {
                argmax_global = decision_and_value.second;
                decision_max = decision_and_value.first;
                best_hyperplan = hyperplan;
            }
        }
        // return std::make_pair(decision_max,best_hyperplan);
        return decision_max;
    }

    Pair<std::shared_ptr<Action>, double> ActionVFMaxplanSerial::selectBestActionKnowingNextHyperplan(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<State> &next_hyperplan, number t)
    {
        auto serial_occupancy_state = state->toOccupancyState();

        auto under_pb = std::dynamic_pointer_cast<SerializedMMDP>(this->world_->getUnderlyingProblem());
        auto agent = under_pb->getAgentId(t);

        // Definie local Variable
        auto decision_rule_value = 0;
        std::shared_ptr<DecisionRule> decision_ = std::make_shared<DeterministicDecisionRule>();

        // Go over all Individual Histories in Serial Occupancy State
        for (const auto &private_history : serial_occupancy_state->getIndividualHistories(agent))
        {
            double argmax_local = -std::numeric_limits<double>::max();

            std::shared_ptr<State> private_serial_occupancy_state; // = serial_occupancy_state->getPrivateOccupancyState(agent,private_history)->toSerialOccupancyState();

            // Go over all action possible for the current agent
            for (const auto &private_action : *under_pb->getActionSpace(agent, t))
            {
                double action_value = this->evaluationOfHyperplanKnowingNextHyperplanAndDiscreteAction(private_serial_occupancy_state, private_action->toAction(), next_hyperplan, t);

                // Take the best deterministic decision rule "decision_" for a precise hyperplan
                if (argmax_local < action_value)
                {
                    argmax_local = action_value;
                    decision_->setProbability(private_history, private_action->toAction(), 1);
                }
            }
            decision_rule_value += serial_occupancy_state->getProbabilityOverIndividualHistories(agent, private_history) * argmax_local;
        }
        return std::make_pair(decision_, decision_rule_value);
    }

    double ActionVFMaxplanSerial::evaluationOfHyperplanKnowingNextHyperplanAndDiscreteAction(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &hyperplan, number t)
    {
        auto under_pb = std::dynamic_pointer_cast<SerializedMPOMDP>(this->world_->getUnderlyingProblem());
        number agent = under_pb->getAgentId(t);

        auto serial_occupancy_state = state->toOccupancyState();
        auto next_hyperplan = hyperplan->toOccupancyState();

        double value = 0;

        // Go over all compressed serial occupancy state
        for (const auto &joint_history : serial_occupancy_state->getJointHistories())
        {
            std::shared_ptr<Action> serial_action = action->toDecisionRule()->act(joint_history->getIndividualHistory(agent));

            for (const auto &hidden_state : serial_occupancy_state->getBeliefsAt(joint_history))
            {
                auto probability = serial_occupancy_state->getProbability(joint_history, hidden_state);

                // Determine the reward
                double immediate_reward = under_pb->getReward(hidden_state, serial_action, t);

                // Compute the next value
                double next_value = 0;

                // Go over all Reachable Serial State
                // for(const auto &next_hidden_state: under_pb->getReachableStates(hidden_state, serial_action,t))
                // {
                //     // Go over all Reachable Observation
                //     for(const auto &serial_observation :under_pb->getReachableObservations(hidden_state, serial_action, next_hidden_state,t))
                //     {
                //         auto joint_serial_observation = std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(serial_observation);

                //         auto history_next = joint_history->expand(joint_serial_observation)->toJointHistory();
                //         next_value += next_hyperplan->getProbability(history_next,next_hidden_state) * under_pb->getDynamics(hidden_state,serial_action,next_hidden_state,serial_observation,t);
                //     }
                // }
                value += probability * (immediate_reward + under_pb->getDiscount(t) * next_value);
            }
        }
        return value;
    }

}