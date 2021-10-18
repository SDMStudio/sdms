#include <sdm/utils/value_function>
#include <sdm/world/base/pomdp_interface.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/utils/value_function/vfunction/pwlc_value_function.hpp>
#include <sdm/core/state/belief_state.hpp>
#include <sdm/core/state/occupancy_state.hpp>

#include <sdm/utils/value_function/update_operator/vupdate/maxplan_update.hpp>

namespace sdm
{

    namespace update
    {
        MaxPlanUpdateOperator::MaxPlanUpdateOperator() {}

        MaxPlanUpdateOperator::MaxPlanUpdateOperator(const std::shared_ptr<SolvableByHSVI> &world, const std::shared_ptr<PWLCValueFunctionInterface> &value_function) : PWLCUpdateOperator(world) {}

        void MaxPlanUpdateOperator::update(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
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

        std::shared_ptr<State> MaxPlanUpdateOperator::computeNewHyperplane(const std::shared_ptr<BeliefInterface> &belief_state, const std::shared_ptr<Action> &action, number t)
        {
            auto under_pb = std::dynamic_pointer_cast<POMDPInterface>(this->world_->getUnderlyingProblem());
            auto belief_mdp = std::static_pointer_cast<BeliefMDP>(this->world_);

            // Creation of a new belief
            auto new_hyperplan = std::make_shared<Belief>();
            new_hyperplan->setDefaultValue(std::static_pointer_cast<PWLCValueFunction>(this->value_function)->getDefaultValue(t));

            // Go over all state in the current belief
            for (const auto &hidden_state : belief->getStates())
            {
                double tmp = 0.0;
                // Go over all hidden state reachable
                for (const auto &next_hidden_state : under_pb->getReachableStates(hidden_state->toState(), action->toAction(), t))
                {
                    // Go over all observation reachable
                    for (const auto &observation : under_pb->getReachableObservations(hidden_state->toState(), action->toAction(), next_hidden_state->toState(), t))
                    {
                        // Determine the next belief conditionning to the action and observation
                        auto tempo_belief = belief_mdp->getNextStateAndProba(belief, action->toAction(), observation->toObservation(), t).first;

                        // Determine the best next hyperplan for the next belief and compute the dynamics and probability of this best next hyperplan
                        tmp += this->value_function->evaluate(tempo_belief, t + 1).first->toBelief()->getProbability(next_hidden_state) * under_pb->getDynamics(hidden_state->toState(), action->toAction(), next_hidden_state->toState(), observation->toObservation(), t);
                    }
                }
                // For each hidden state with associate the value r(x,u) + discount* \sum_{x_,z_} p(x,u,z_,x_)* best_next_hyperplan(x_);
                new_hyperplan->setProbability(hidden_state->toState(), under_pb->getReward(hidden_state->toState(), action->toAction(), t) + this->world_->getDiscount(t) * tmp);
            }
            new_hyperplan->finalize();
            return new_hyperplan;
        }

        std::shared_ptr<State> MaxPlanUpdateOperator::computeNewHyperplane(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t)
        {
            try
            {
                auto under_pb = std::dynamic_pointer_cast<POMDPInterface>(this->world_->getUnderlyingProblem());
                auto occupancy_mdp = std::static_pointer_cast<OccupancyMDP>(this->world_);

                // Create the new hyperplan
                auto new_hyperplan = std::make_shared<OccupancyState>(under_pb->getNumAgents());
                new_hyperplan->setDefaultValue(std::static_pointer_cast<PWLCValueFunction>(this->value_function)->getDefaultValue(t));

                // Get the next occupancy state associated to the decision rule
                auto next_occupancy_state = occupancy_mdp->getNextStateAndProba(occupancy, decision_rule, sdm::NO_OBSERVATION, t).first;

                // Determine the best next hyperplan associated to the next occupancy state
                auto best_evaluate_occupancy_state = this->value_function->evaluate(next_occupancy_state, t + 1).first->toOccupancyState();

                // Go over all joint history for the occupancy state
                for (const auto &joint_history : occupancy_state->getFullyUncompressedOccupancy()->getJointHistories())
                {
                    // Select the joint action
                    auto action = occupancy_mdp->applyDecisionRule(occupancy_state, occupancy_state->getCompressedJointHistory(joint_history), decision_rule, t);

                    // Create new belief
                    auto new_belief = std::make_shared<Belief>();
                    for (const auto &hidden_state : occupancy_state->getFullyUncompressedOccupancy()->getBeliefAt(joint_history)->getStates())
                    {
                        double tmp = 0.0;
                        // Go over all hidden state reachable
                        for (const auto &next_hidden_state : under_pb->getReachableStates(hidden_state, action, t))
                        {
                            // Go over all observation reachable
                            for (const auto &observation : under_pb->getReachableObservations(hidden_state, action, next_hidden_state, t))
                            {
                                // Expand the current joint history
                                auto next_joint_history = joint_history->expand(observation->toObservation())->toJointHistory();
                                tmp += best_evaluate_occupancy_state->getProbability(next_joint_history, next_hidden_state) * under_pb->getDynamics(hidden_state, action, next_hidden_state, observation, t);
                            }
                        }
                        // For each hidden state with associate the value r(x,u) + discount* \sum_{x_,z_} p(x,u,z_,x_)* best_next_hyperplan(x_);
                        new_belief->setProbability(hidden_state, under_pb->getReward(hidden_state, action, t) + this->world_->getDiscount(t) * tmp);
                    }
                    new_belief->finalize();
                    new_hyperplan->setProbability(joint_history, new_belief, 1);
                }
                new_hyperplan->finalize();
                return new_hyperplan;
            }
            catch (const std::exception &e)
            {
                std::cerr << "MaxPlanUpdateOperator::setHyperplanOccupancy error " << e.what() << '\n';
                exit(-1);
            }
        }
    }
}