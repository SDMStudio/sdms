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

    ActionSelectionMaxplanSerial::ActionSelectionMaxplanSerial(const std::shared_ptr<SolvableByDP> &world) : MaxPlanSelectionBase(world)
    {
        if (auto derived = std::dynamic_pointer_cast<SerialProblemInterface>(world))
            this->serial_mpomdp = derived;
        else
            throw sdm::exception::TypeError("Action maxplan serial is only available for worlds inheriting from 'SerialProblemInterface'.");
    }

    std::shared_ptr<SerialProblemInterface> ActionSelectionMaxplanSerial::getSerialProblem() const
    {
        return this->serial_mpomdp;
    }

    Pair<std::shared_ptr<Action>, double> ActionSelectionMaxplanSerial::computeGreedyActionAndValue(const std::shared_ptr<ValueFunctionInterface> &value_function, const std::shared_ptr<State> &state, const std::shared_ptr<BeliefInterface> &hyperplane, number t)
    {
        auto serial_occupancy_state = std::dynamic_pointer_cast<OccupancyState>(state);
        auto agent_id = this->getSerialProblem()->getAgentId(t);

        // Definie local Variable
        double decision_rule_value = 0.0;
        std::shared_ptr<DecisionRule> decision_rule = std::make_shared<DeterministicDecisionRule>();

        // Go over all Individual Histories in Serial Occupancy State
        for (const auto &individual_history : serial_occupancy_state->getIndividualHistories(agent_id))
        {
            // Select the best action to execute in this context and get the value
            auto [best_action, value] = this->selectBestAction(value_function, serial_occupancy_state, hyperplane, individual_history, t);
            // Update the decision rule with new history and action
            decision_rule->setProbability(individual_history, best_action, 1);
            // Increment the total value of the decision rule
            decision_rule_value += value;
        }
        return std::make_pair(decision_rule, decision_rule_value);
    }

    Pair<std::shared_ptr<Action>, double> ActionSelectionMaxplanSerial::selectBestAction(const std::shared_ptr<ValueFunctionInterface> &value_function, const std::shared_ptr<OccupancyState> &occupancy_state, const std::shared_ptr<BeliefInterface> &next_hyperplane, const std::shared_ptr<HistoryInterface> &ihistory, number t)
    {
        std::shared_ptr<Action> best_action;
        double action_value, argmax_local = std::numeric_limits<double>::lowest();

        auto agent_id = this->getSerialProblem()->getAgentId(t);
        std::shared_ptr<PrivateOccupancyState> private_serial_occupancy_state = occupancy_state->getPrivateOccupancyState(agent_id, ihistory);

        double probability = occupancy_state->getProbabilityOverIndividualHistories(agent_id, ihistory);

        // Go over all action possible for the current agent_id
        for (const auto &private_action : *this->getWorld()->getUnderlyingProblem()->getActionSpace(t))
        {
            // Evaluate the value of executing this action in the given state & history
            action_value = this->evaluateAction(value_function, private_serial_occupancy_state, private_action->toAction(), next_hyperplane, t);

            // Take the best deterministic decision rule "decision_" for a precise hyperplan
            if (argmax_local < action_value)
            {
                argmax_local = action_value;
                best_action = private_action->toAction();
            }
        }
        return {best_action, probability * argmax_local};
    }

    double ActionSelectionMaxplanSerial::evaluateAction(const std::shared_ptr<ValueFunctionInterface> &value_function, const std::shared_ptr<OccupancyState> &occupancy_state, const std::shared_ptr<Action> &action, const std::shared_ptr<BeliefInterface> &next_hyperplane, number t)
    {
        double value = 0.0;
        double discount = this->getWorld()->getDiscount(t);

        // Go over all joint history in the subsequent private occupancy state (knowing the information individual history)
        for (const auto &joint_history : occupancy_state->getJointHistories())
        {
             value += this->getWeight(value_function, occupancy_state, joint_history, action, next_hyperplane, t);
        }
        return value;
    }
}