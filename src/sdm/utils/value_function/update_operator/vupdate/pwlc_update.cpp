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

        void PWLCUpdate::update(const std::shared_ptr<State> &state, number t)
        {
            std::shared_ptr<Action> action = this->getValueFunction()->getGreedyAction(state, t);
            this->update(state, action, t);
        }


        void PWLCUpdate::update(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
        {
            if (sdm::isInstanceOf<OccupancyStateInterface>(state))
            {
                this->getValueFunction()->addHyperplaneAt(state, this->computeNewHyperplane(state->toOccupancyState(), action, t), t);
            }
            else if (sdm::isInstanceOf<BeliefInterface>(state))
            {
                this->getValueFunction()->addHyperplaneAt(state, this->computeNewHyperplane(state->toBelief(), t), t);
            }
        }

        std::shared_ptr<State> PWLCUpdate::computeNewHyperplane(const std::shared_ptr<BeliefInterface> &belief_state, number t)
        {
            auto pomdp = std::dynamic_pointer_cast<POMDPInterface>(this->getWorld()->getUnderlyingProblem());

            // Compute \alpha_ao (\beta_ao in the paper of Trey Smith)
            RecursiveMap<std::shared_ptr<Action>, std::shared_ptr<Observation>, std::shared_ptr<BeliefInterface>> alpha_ao;
            for (const auto &action : *getWorld()->getActionSpaceAt(belief_state, t))
            {
                for (const auto &observation : *getWorld()->getObservationSpaceAt(belief_state, action->toAction(), t))
                {
                    auto next_belief_state = getWorld()->getNextStateAndProba(belief_state, action->toAction(), observation->toObservation(), t).first;
                    alpha_ao[action->toAction()][observation->toObservation()] = this->getValueFunction()->getHyperplaneAt(next_belief_state, t+1)->toBelief();
                }
            }

            // Creation of a new belief
            double best_value = std::numeric_limits<double>::lowest(), alpha_a_value;
            auto new_hyperplane = std::make_shared<Belief>();
            new_hyperplane->setDefaultValue(this->getValueFunction()->getDefaultValue(t));
            for (const auto &action : *getWorld()->getActionSpaceAt(belief_state, t))
            {
                // Creation of a new belief
                auto alpha_a = std::make_shared<Belief>();
                alpha_a->setDefaultValue(this->getValueFunction()->getDefaultValue(t));

                // Go over all state in the current belief
                for (const auto &state : belief_state->getStates())
                {
                    double next_expected_value = 0.0;

                    // Go over all hidden state reachable next state
                    for (const auto &next_state : pomdp->getReachableStates(state, action->toAction(), t))
                    {
                        // Go over all observation reachable observation
                        for (const auto &observation : pomdp->getReachableObservations(state, action->toAction(), next_state, t))
                        {

                            // Get the next value of an hyperplane
                            double next_alpha_value = alpha_ao[action->toAction()][observation]->getVectorInferface()->getValueAt(next_state);

                            // Determine the best next hyperplan for the next belief and compute the dynamics and probability of this best next hyperplan
                            next_expected_value += next_alpha_value * pomdp->getDynamics(state, action->toAction(), next_state, observation, t);
                        }
                    }
                    // For each hidden state with associate the value \beta^{new}(x) = r(x,u) + \gamma * \sum_{x_,z_} p(x,u,z_,x_) * best_next_hyperplan(x_);
                    alpha_a->setValueAt(state, pomdp->getReward(state, action->toAction(), t) + this->getWorld()->getDiscount(t) * next_expected_value);
                }
                alpha_a->finalize();

                alpha_a_value = belief_state->operator^(alpha_a);
                if (alpha_a_value > best_value)
                {
                    new_hyperplane = alpha_a;
                    best_value = alpha_a_value;
                }
            }
            return new_hyperplane;
        }

        std::shared_ptr<State> PWLCUpdate::computeNewHyperplane(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<Action>& decision_rule, number t)
        {
            auto pomdp = std::dynamic_pointer_cast<POMDPInterface>(this->getWorld()->getUnderlyingProblem());
            auto occupancy_mdp = std::dynamic_pointer_cast<OccupancyMDP>(this->getWorld());

            // Create the new hyperplan
            auto new_hyperplan = std::make_shared<OccupancyState>(pomdp->getNumAgents());
            new_hyperplan->setDefaultValue(this->getValueFunction()->getDefaultValue(t));

            // Get the next occupancy state associated to the decision rule
            auto next_occupancy_state = occupancy_mdp->getNextStateAndProba(occupancy_state, decision_rule, sdm::NO_OBSERVATION, t).first;

            // Determine the best next hyperplan associated to the next occupancy state
            auto best_next_hyperplane = std::dynamic_pointer_cast<ValueFunction>(this->getValueFunction())->evaluate(next_occupancy_state, t + 1).first;

            // Go over all joint history for the occupancy state
            for (const auto &jhistory : occupancy_state->getFullyUncompressedOccupancy()->getJointHistories())
            {
                // Select the joint action
                auto action = occupancy_mdp->applyDecisionRule(occupancy_state, occupancy_state->getCompressedJointHistory(jhistory), decision_rule, t);

                // Create new belief
                auto new_belief = std::make_shared<Belief>();
                for (const auto &state : occupancy_state->getFullyUncompressedOccupancy()->getBeliefAt(jhistory)->getStates())
                {
                    // For each hidden state with associate the value r(x,u) + discount* \sum_{x_,z_} p(x,u,z_,x_)* best_next_hyperplan(x_);
                    new_belief->setProbability(state, this->getValueFunction()->getBeta(best_next_hyperplane, state, jhistory, action, t));
                }
                new_belief->finalize();
                new_hyperplan->setProbability(jhistory, new_belief, 1);
            }
            new_hyperplan->finalize(false);
            return new_hyperplan;
        }

    }
}