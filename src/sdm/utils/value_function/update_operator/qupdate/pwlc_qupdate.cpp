#include <sdm/utils/value_function/update_operator/qupdate/pwlc_qupdate.hpp>
#include <sdm/core/state/belief_state.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/world/base/pomdp_interface.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/utils/value_function/pwlc_value_function_interface.hpp>
#include <sdm/utils/value_function/qfunction/pwlc_qvalue_function.hpp>

namespace sdm
{
    namespace update
    {
        PWLCQUpdate::PWLCQUpdate(
            std::shared_ptr<ExperienceMemory> experience_memory,
            std::shared_ptr<ValueFunctionInterface> q_value,
            std::shared_ptr<ValueFunctionInterface> target_q_value,
            double learning_rate) : PWLCQUpdateOperator(experience_memory, q_value, target_q_value, learning_rate)
        {
        }

        double PWLCQUpdate::target(const std::shared_ptr<State> &state,
                                   const std::shared_ptr<Action> &action,
                                   double reward,
                                   const std::shared_ptr<State> &next_state,
                                   const std::shared_ptr<Action> &next_action,
                                   number t)
        {
            return reward + getWorld()->getDiscount(t) * q_value->getQValueAt(next_state, next_action, t + 1);
        }

        double PWLCQUpdate::delta(const std::shared_ptr<State> &state,
                                  const std::shared_ptr<Action> &action,
                                  double reward,
                                  const std::shared_ptr<State> &next_state,
                                  const std::shared_ptr<Action> &next_action,
                                  number t)
        {
            double target_value = target(state, action, reward, next_state, next_action, t);
            return (target_value - q_value->getQValueAt(state, action, t));
        }

        void PWLCQUpdate::update(number t)
        {
            auto [state, action, reward, next_state, next_action] = this->experience_memory->sample(t)[0];
            double delta = this->delta(state->toState(), action, reward, next_state->toState(), next_action, t);

            // Update the hyperplane of the ball that include the sampled state
            if (sdm::isInstanceOf<OccupancyStateInterface>(state))
            {
                auto new_hyperplane = this->computeNewHyperplane(state->toState()->toOccupancyState(), action->toDecisionRule(), delta, t);
                q_value->addHyperplaneAt(state->toState(), new_hyperplane, t);
            }
            else
            {
                throw sdm::exception::TypeError("TypeError: state must derive from OccupancyStateInterface");
            }
            // else if (sdm::isInstanceOf<BeliefInterface>(state))
            // {
            //     this->qvalue_function->addHyperplaneAt(state->toState(), this->computeNewHyperplane(state->toState()->toBelief(), action,delta, t), t);
            // }
        }

        std::shared_ptr<State> PWLCQUpdate::computeNewHyperplane(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<DecisionRule> &decision_rule, double delta, number t)
        {
            // \beta(x,o,u) =  \beta(x,o,u) + \alpha * \delta * \xi(x,o) * a(u | o)
            auto pomdp = std::dynamic_pointer_cast<POMDPInterface>(this->getWorld()->getUnderlyingProblem());
            auto occupancy_mdp = std::dynamic_pointer_cast<OccupancyMDP>(this->getWorld());

            // Get old hyperplane
            auto old_hyperplane = std::dynamic_pointer_cast<PWLCQValueFunction::Hyperplane>(q_value->getHyperplaneAt(occupancy_state, t));

            // Instanciate new hyperplane
            auto new_hyperplane = std::make_shared<PWLCQValueFunction::Hyperplane>();
            new_hyperplane->state.setDefault(q_value->getDefaultValue(t));

            // Go over all joint history for the occupancy state
            for (const auto &history : occupancy_state->getFullyUncompressedOccupancy()->getJointHistories())
            {
                // Select the joint action
                auto action = occupancy_mdp->applyDecisionRule(occupancy_state, occupancy_state->getCompressedJointHistory(history), decision_rule, t);

                // Go over all states in the belief
                for (const auto &state : occupancy_state->getFullyUncompressedOccupancy()->getBeliefAt(history)->getStates())
                {
                    // For each hidden state with associate the value r(x,u) + discount* \sum_{x_,z_} p(x,u,z_,x_)* best_next_hyperplan(x_);
                    double new_value = q_value->getBeta(old_hyperplane, state, history, action, t) + learning_rate * delta * occupancy_state->getProbability(history, state); //* decision_rule->toDecisionRule()->getProbability(occupancy_state->getCompressedJointHistory(history), action);
                    new_hyperplane->state.setValueAt(std::tuple(state, history, action), new_value);
                }
            }
            new_hyperplane->state.finalize();

            return new_hyperplane;
        }
    }
}