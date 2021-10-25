#ifdef WITH_CPLEX

#include <sdm/utils/value_function/action_selection/action_maxplan_serial.hpp>
#include <sdm/core/action/det_decision_rule.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>
#include <sdm/world/serial_mpomdp.hpp>


#include <sdm/world/occupancy_mdp.hpp>

namespace sdm
{
    ActionSelectionMaxplanSerial::ActionSelectionMaxplanSerial() {}

    ActionSelectionMaxplanSerial::ActionSelectionMaxplanSerial(const std::shared_ptr<SolvableByHSVI> &world) : MaxPlanSelectionBase(world) {}

    Pair<std::shared_ptr<Action>,double> ActionSelectionMaxplanSerial::computeGreedyActionAndValue(const std::shared_ptr<ValueFunctionInterface> &value_function, const std::shared_ptr<State> &state, number t)
    {
        return this->selectBestDecisionRuleKnowingNextHyperplan(value_function, state, this->tmp_representation, t);
    }

    Pair<std::shared_ptr<Action>, double> ActionSelectionMaxplanSerial::selectBestDecisionRuleKnowingNextHyperplan(const std::shared_ptr<ValueFunctionInterface> &, const std::shared_ptr<State> &state, const std::shared_ptr<State> &next_hyperplan, number t)
    {
        auto serial_occupancy_state = state->toOccupancyState();
        auto under_pb = std::dynamic_pointer_cast<SerialMMDPInterface>(this->world_->getUnderlyingProblem());
        auto agent = under_pb->getAgentId(t);

        // Definie local Variable
        double decision_rule_value = 0.0;
        std::shared_ptr<DecisionRule> decision_rule = std::make_shared<DeterministicDecisionRule>();

        // Go over all Individual Histories in Serial Occupancy State
        for (const auto &individual_history : serial_occupancy_state->getIndividualHistories(agent))
        {
            // Select the best action to execute in this context and get the value
            auto [best_action, value] = this->selectBestActionKnowingNextHyperplanAndHistory(serial_occupancy_state, next_hyperplan, individual_history, t);
            // Update the decision rule with new history and action
            decision_rule->setProbability(individual_history, best_action, 1);
            // Increment the total value of the decision rule
            decision_rule_value += value;
        }
        return std::make_pair(decision_rule, decision_rule_value);
    }

    Pair<std::shared_ptr<Action>, double> ActionSelectionMaxplanSerial::selectBestActionKnowingNextHyperplanAndHistory(const std::shared_ptr<State> &state, const std::shared_ptr<State> &next_hyperplan, const std::shared_ptr<HistoryInterface> &ihistory, number t)
    {
        auto under_pb = std::dynamic_pointer_cast<SerialMPOMDPInterface>(this->world_->getUnderlyingProblem());
        
        auto agent = under_pb->getAgentId(t);
        auto occupancy_state = state->toOccupancyState();
        std::shared_ptr<PrivateOccupancyState> private_serial_occupancy_state = std::dynamic_pointer_cast<OccupancyState>(state)->getPrivateOccupancyState(under_pb->getAgentId(t), ihistory);

        std::shared_ptr<Action> best_action;
        double argmax_local = -std::numeric_limits<double>::max();

        double probability = occupancy_state->getProbabilityOverIndividualHistories(agent, ihistory);

        // Go over all action possible for the current agent
        for (const auto &private_action : *under_pb->getActionSpace(t))
        {
            // Evaluate the value of executing this action in the given state & history
            double action_value = probability * this->evaluationOfHyperplanKnowingNextHyperplanAndDiscreteAction(private_serial_occupancy_state, private_action->toAction(), next_hyperplan, t);

            // Take the best deterministic decision rule "decision_" for a precise hyperplan
            if (argmax_local < action_value)
            {
                argmax_local = action_value;
                best_action = private_action->toAction();
            }
        }
        return {best_action, argmax_local};
    }

    double ActionSelectionMaxplanSerial::evaluationOfHyperplanKnowingNextHyperplanAndDiscreteAction(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_hyperplan, number t)
    {
        auto occupancy_state = state->toOccupancyState();
        auto under_pb = std::dynamic_pointer_cast<SerialMPOMDPInterface>(this->world_->getUnderlyingProblem());

        double value = 0.0;
        double discount = under_pb->getDiscount(t);

        // Go over all joint history in the subsequent private occupancy state (knowing the information individual history)
        for (const auto &joint_history : occupancy_state->getJointHistories())
        {
            // Go over all hidden state in the subsequent private occupancy state (knowing the information individual history and joint history)
            for (const auto &state : occupancy_state->getBeliefAt(joint_history)->getStates())
            {
                //Get the probability of the occupancy state
                auto probability = occupancy_state->getProbability(joint_history, state);
                
                // Determine the reward
                double immediate_reward = under_pb->getReward(state, action, t);
                double next_expected_value = this->evaluateNextExpectedValueAt(next_hyperplan, joint_history, state, action, t);

                value += probability * (immediate_reward + discount * next_expected_value);
            }
        }
        return value;
    }

    double ActionSelectionMaxplanSerial::evaluateNextExpectedValueAt(const std::shared_ptr<State> &hyperplan, const std::shared_ptr<HistoryInterface> &joint_history, const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        auto next_hyperplan = hyperplan->toOccupancyState();
        auto under_pb = std::dynamic_pointer_cast<SerialMPOMDPInterface>(this->world_->getUnderlyingProblem());

        // Compute the next value
        double next_expected_value = 0.0;

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

#endif