#include <sdm/core/state/belief_state.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/world/base/pomdp_interface.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/utils/value_function/pwlc_value_function_interface.hpp>

#include <sdm/utils/value_function/update_operator/vupdate/pwlc_update.hpp>

namespace sdm
{

    namespace update
    {
        PWLCUpdate::PWLCUpdate(const std::shared_ptr<ValueFunctionInterface> &value_function)
            : PWLCUpdateOperator(value_function) {}

        void PWLCUpdate::update(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
        {
            if (sdm::isInstanceOf<OccupancyStateInterface>(state))
            {
                this->value_function->addHyperplaneAt(state, this->computeNewHyperplane(state->toOccupancyState(), action, t), t);
            }
            else if (sdm::isInstanceOf<BeliefInterface>(state))
            {
                this->value_function->addHyperplaneAt(state, this->computeNewHyperplane(state->toBelief(), action, t), t);
            }
        }

        std::shared_ptr<State> PWLCUpdate::computeNewHyperplane(const std::shared_ptr<BeliefInterface> &belief_state, const std::shared_ptr<Action> &action, number t)
        {
            // Creation of a new belief
            auto new_hyperplan = std::make_shared<Belief>();
            new_hyperplan->setDefaultValue(value_function->getDefaultValue(t));

            // Go over all state in the current belief
            for (const auto &state : belief_state->getStates())
            {
                // For each hidden state with associate the value \beta^{new}(x) = r(x,u) + \gamma * \sum_{x_,z_} p(x,u,z_,x_) * best_next_hyperplan(x_);
                new_hyperplan->setValueAt(state, value_function->getBeta(nullptr, state, nullptr, action, t));
            }
            new_hyperplan->finalize();
            return new_hyperplan;
        }

        std::shared_ptr<State> PWLCUpdate::computeNewHyperplane(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t)
        {
            auto pomdp = std::dynamic_pointer_cast<POMDPInterface>(this->getWorld()->getUnderlyingProblem());
            auto occupancy_mdp = std::dynamic_pointer_cast<OccupancyMDP>(this->getWorld());

            // Create the new hyperplan
            auto new_hyperplan = std::make_shared<OccupancyState>(pomdp->getNumAgents());
            new_hyperplan->setDefaultValue(value_function->getDefaultValue(t));

            // Get the next occupancy state associated to the decision rule
            auto next_occupancy_state = occupancy_mdp->getNextStateAndProba(occupancy_state, decision_rule, sdm::NO_OBSERVATION, t).first;
            
            // Determine the best next hyperplan associated to the next occupancy state
            auto best_evaluate_occupancy_state = std::dynamic_pointer_cast<ValueFunction>(this->getValueFunction())->evaluate(next_occupancy_state, t + 1).first;

            // Go over all joint history for the occupancy state
            for (const auto &jhistory : occupancy_state->getFullyUncompressedOccupancy()->getJointHistories())
            {
                // Select the joint action
                auto action = occupancy_mdp->applyDecisionRule(occupancy_state->toOccupancyState(), occupancy_state->toOccupancyState()->getCompressedJointHistory(jhistory), decision_rule, t);

                // Create new belief
                auto new_belief = std::make_shared<Belief>();
                for (const auto &state : occupancy_state->getFullyUncompressedOccupancy()->getBeliefAt(jhistory)->getStates())
                {
                    // For each hidden state with associate the value r(x,u) + discount* \sum_{x_,z_} p(x,u,z_,x_)* best_next_hyperplan(x_);
                    new_belief->setProbability(state, value_function->getBeta(best_evaluate_occupancy_state, state, jhistory, action, t));
                }
                new_belief->finalize();
                new_hyperplan->setProbability(jhistory, new_belief, 1);
            }
            new_hyperplan->finalize();
            return new_hyperplan;
        }

    }
}