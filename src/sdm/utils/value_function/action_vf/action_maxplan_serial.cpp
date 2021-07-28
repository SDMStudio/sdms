#include <sdm/utils/value_function/action_vf/action_maxplan_serial.hpp>
#include <sdm/core/action/det_decision_rule.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>
#include <sdm/world/serialized_mpomdp.hpp>

namespace sdm
{
    ActionVFMaxplanSerial::ActionVFMaxplanSerial() {}

    ActionVFMaxplanSerial::ActionVFMaxplanSerial(const std::shared_ptr<SolvableByHSVI> &world) : ActionVFBase(world) {}

    Pair<std::shared_ptr<Action>,double> ActionVFMaxplanSerial::selectBestAction(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t)
    {
        // Definie local Variable
        double argmax_global = -std::numeric_limits<double>::max();
        std::shared_ptr<Action> decision_max;
        std::shared_ptr<State> best_hyperplan;

        // // Go other the hyperplanes of decision step t+1
        for (const auto &hyperplan : vf->getSupport(t + 1))
        {
            // Compute the best action and corresponding value for a hyperplan
            auto decision_and_value = this->selectBestDecisionRuleKnowingNextHyperplan(vf, state, hyperplan, t);

            // Take the best deterministic decision rule and best hyperplan associated
            if (argmax_global < decision_and_value.second)
            {
                argmax_global = decision_and_value.second;
                decision_max = decision_and_value.first;
                best_hyperplan = hyperplan;
            }
        }
        // return std::make_pair(decision_max,best_hyperplan);
        return {decision_max, argmax_global};
    }

    Pair<std::shared_ptr<Action>, double> ActionVFMaxplanSerial::selectBestDecisionRuleKnowingNextHyperplan(const std::shared_ptr<ValueFunction> &, const std::shared_ptr<State> &state, const std::shared_ptr<State> &next_hyperplan, number t)
    {
        auto serial_occupancy_state = state->toOccupancyState();
        auto under_pb = std::dynamic_pointer_cast<SerialMMDPInterface>(this->world_->getUnderlyingProblem());
        auto agent = under_pb->getAgentId(t);

        // Definie local Variable
        auto decision_rule_value = 0;
        std::shared_ptr<DecisionRule> decision_rule = std::make_shared<DeterministicDecisionRule>();

        // Go over all Individual Histories in Serial Occupancy State
        for (const auto &individual_history : serial_occupancy_state->getIndividualHistories(agent))
        {
            // Select the best action to execute in this context and get the value
            auto [best_action, value] = this->selectBestActionKnowingNextHyperplanAndHistory(serial_occupancy_state, next_hyperplan, individual_history, t);
            // Update the decision rule with new history and action
            decision_rule->setProbability(individual_history, best_action, 1);
            // Increment the total value of the decision rule
            decision_rule_value += serial_occupancy_state->getProbabilityOverIndividualHistories(agent, individual_history) * value;
        }
        return std::make_pair(decision_rule, decision_rule_value);
    }

    Pair<std::shared_ptr<Action>, double> ActionVFMaxplanSerial::selectBestActionKnowingNextHyperplanAndHistory(const std::shared_ptr<State> &state, const std::shared_ptr<State> &next_hyperplan, const std::shared_ptr<HistoryInterface> &ihistory, number t)
    {
        auto under_pb = std::dynamic_pointer_cast<SerialMMDPInterface>(this->world_->getUnderlyingProblem());

        std::shared_ptr<Action> best_action;
        double argmax_local = -std::numeric_limits<double>::max();

        std::shared_ptr<PrivateOccupancyState> private_serial_occupancy_state = std::dynamic_pointer_cast<OccupancyState>(state)->getPrivateOccupancyState(under_pb->getAgentId(t), ihistory);

        // Go over all action possible for the current agent
        for (const auto &private_action : *under_pb->getActionSpace(t))
        {
            // Evaluate the value of executing this action in the given state & history
            double action_value = this->evaluationOfHyperplanKnowingNextHyperplanAndDiscreteAction(private_serial_occupancy_state, private_action->toAction(), next_hyperplan, t);

            // Take the best deterministic decision rule "decision_" for a precise hyperplan
            if (argmax_local < action_value)
            {
                argmax_local = action_value;
                best_action = private_action->toAction();
            }
        }
        return {best_action, argmax_local};
    }

    double ActionVFMaxplanSerial::evaluationOfHyperplanKnowingNextHyperplanAndDiscreteAction(const std::shared_ptr<PrivateOccupancyState> &private_occupancy_state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_hyperplan, number t)
    {
        auto under_pb = std::dynamic_pointer_cast<SerialMPOMDPInterface>(this->world_->getUnderlyingProblem());
        // number agent = under_pb->getAgentId(t);

        double value = 0;

        // Go over all compressed serial occupancy state
        for (const auto &joint_history : private_occupancy_state->getJointHistories())
        {
            for (const auto &state : private_occupancy_state->getBeliefAt(joint_history)->getStates())
            {
                //
                auto probability = private_occupancy_state->getProbability(joint_history, state);
                // Determine the reward
                double immediate_reward = under_pb->getReward(state, action, t);
                //
                double next_expected_value = this->evaluateNextExpectedValueAt(next_hyperplan, joint_history, state, action, t);

                value += probability * (immediate_reward + under_pb->getDiscount(t) * next_expected_value);
            }
        }
        return value;
    }

    double ActionVFMaxplanSerial::evaluateNextExpectedValueAt(const std::shared_ptr<State> &hyperplan, const std::shared_ptr<HistoryInterface> &joint_history, const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        auto next_hyperplan = hyperplan->toOccupancyState();
        auto under_pb = std::dynamic_pointer_cast<SerialMPOMDPInterface>(this->world_->getUnderlyingProblem());

        // Compute the next value
        double next_expected_value = 0;

        // Go over all Reachable Serial State
        for (const auto &next_state : under_pb->getReachableStates(state, action, t))
        {
            // Go over all Reachable Observation
            for (const auto &next_observation : under_pb->getReachableObservations(state, action, next_state, t))
            {
                auto next_joint_history = joint_history->expand(next_observation)->toJointHistory();
                next_expected_value += next_hyperplan->getProbability(next_joint_history, next_state) * under_pb->getDynamics(state, action, next_state, next_observation, t);
            }
        }
        return next_expected_value;
    }
}