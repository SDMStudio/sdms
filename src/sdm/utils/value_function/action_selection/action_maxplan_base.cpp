
#include <sdm/utils/value_function/action_selection/action_maxplan_base.hpp>
#include <sdm/utils/value_function/value_function_interface.hpp>
#include <sdm/utils/value_function/pwlc_value_function_interface.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>

namespace sdm
{

    MaxPlanSelectionBase::MaxPlanSelectionBase()
    {
    }

    MaxPlanSelectionBase::MaxPlanSelectionBase(const std::shared_ptr<SolvableByDP> &world) : ActionSelectionBase(world)
    {
    }

    Pair<std::shared_ptr<Action>, double> MaxPlanSelectionBase::getGreedyActionAndValue(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, number t)
    {
        // Cast the generic value function into a piece-wise linear convex value function.
        this->pwlc_vf = std::dynamic_pointer_cast<PWLCValueFunctionInterface>(vf);

        // Instanciate return variables
        std::shared_ptr<Action> max_decision_rule;
        double max_value = -std::numeric_limits<double>::max();

        // Go over all hyperplan in the Support
        for (const auto &hyperplane : this->pwlc_vf.lock()->getHyperplanesAt(state, t + 1))
        {
            // Compute the greedy action and value for a given hyperplan
            auto pair_action_value = this->computeGreedyActionAndValue(this->pwlc_vf.lock(), state, hyperplane->toBelief(), t);

            // Select the Best Action
            if (pair_action_value.second > max_value)
            {
                max_decision_rule = pair_action_value.first;
                max_value = pair_action_value.second;
            }
        }
        return {max_decision_rule, max_value};
    }

    double MaxPlanSelectionBase::getWeight(const std::shared_ptr<ValueFunctionInterface> &value_function, const std::shared_ptr<OccupancyStateInterface>& occupancy_state, const std::shared_ptr<JointHistoryInterface>& joint_history, const std::shared_ptr<Action>& action, const std::shared_ptr<BeliefInterface> &hyperplane, number t)
    {
        // Compute \sum_{x} s(o,x)* [ r(x,u) + discount * \sum_{x_,z_} p(x,u,x_,z_,) * next_hyperplan(<o,z_>,x_)]
        double weight = 0.0;

        // Go over all hidden state in the belief conditionning to a joint history
        for (const auto &state : occupancy_state->getBeliefAt(joint_history)->getStates())
        {
            weight += occupancy_state->getProbability(joint_history, state) * this->pwlc_vf.lock()->getBeta(hyperplane, state, joint_history, action, t);
        }
        return weight;
    }
} // namespace sdm
