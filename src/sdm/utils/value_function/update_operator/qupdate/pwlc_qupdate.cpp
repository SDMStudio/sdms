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
            std::shared_ptr<ValueFunctionInterface> target_q_value) : PWLCQUpdateOperator(experience_memory, q_value, target_q_value)
        {
        }

        double PWLCQUpdate::target(const std::shared_ptr<State> &state,
                                   const std::shared_ptr<Action> &action,
                                   double reward,
                                   const std::shared_ptr<State> &next_state,
                                   const std::shared_ptr<Action> &next_action,
                                   number t)
        {
            return reward + getWorld()->getDiscount(t) * this->getQValueFunction()->getQValueAt(next_state, next_action, t + 1);
        }

        double PWLCQUpdate::delta(const std::shared_ptr<State> &state,
                                  const std::shared_ptr<Action> &action,
                                  double reward,
                                  const std::shared_ptr<State> &next_state,
                                  const std::shared_ptr<Action> &next_action,
                                  number t)
        {
            double target_value = target(state, action, reward, next_state, next_action, t);
            double delta = (target_value - this->getQValueFunction()->getQValueAt(state, action, t));
            return delta;
        }

        void PWLCQUpdate::update(double learning_rate, number t)
        {
            // auto [state, action, reward, next_state, next_action] = this->experience_memory->sample(t)[0];
            // double delta = this->delta(state, action, reward, next_state, next_action, t);
            // this->updateHyperplane(state->toOccupancyState(), action->toDecisionRule(), delta, learning_rate, t);
            this->updateHyperplane(learning_rate, t);
        }

        void PWLCQUpdate::updateHyperplane(double learning_rate, number t)
        {
            auto [state, action, reward, next_state, next_action] = this->experience_memory->sample(t)[0];

            auto hyperplane = std::static_pointer_cast<Hyperplane>(this->getQValueFunction()->getHyperplaneAt(state, t));
            auto hyperplane_ = std::static_pointer_cast<Hyperplane>(this->getQValueFunction()->getHyperplaneAt(next_state, t + 1));

            auto s = state->toOccupancyState();
            auto s_ = std::dynamic_pointer_cast<OccupancyState>(next_state);
            auto a = action->toDecisionRule(), a_ = next_action->toDecisionRule();

            auto occupancy_mdp = std::dynamic_pointer_cast<OccupancyMDP>(this->getWorld());
            auto pomdp = std::dynamic_pointer_cast<POMDPInterface>(this->getWorld()->getUnderlyingProblem());

            for (auto o : s->getJointHistories())
            {
                // auto &&c_o = s->getCompressedJointHistory(o);

                auto u = occupancy_mdp->applyDecisionRule(s, o, a, t);

                for (auto x : s->getBeliefAt(o)->getStates())
                {
                    double delta_xou = pomdp->getReward(x, u, t);

                    if (t + 1 < pomdp->getHorizon())
                    {
                        for (const auto &x_ : pomdp->getReachableStates(x, u, t))
                        {
                            for (const auto &z : pomdp->getReachableObservations(x, u, x_, t))
                            {
                                // set next-step history h' = h + u + z
                                auto o_ = o->expand(std::static_pointer_cast<JointObservation>(z));
                                auto &&c_o_ = s_->getCompressedJointHistory(o_);

                                if (s_->getProbability(c_o_) == 0)
                                    continue;

                                auto u_ = occupancy_mdp->applyDecisionRule(s_, c_o_, a_, t + 1);

                                if (u_ == nullptr)
                                    continue;
                                delta_xou += getWorld()->getDiscount(t) * pomdp->getDynamics(x, u, x_, z, t) * hyperplane_->getValueAt(c_o_, x_, u_);
                            }
                        }
                    }
                    hyperplane->setValueAt(o, x, u, hyperplane->getValueAt(o, x, u) + learning_rate * (delta_xou - hyperplane->getValueAt(o, x, u)));
                }
            }
        }

        void PWLCQUpdate::updateHyperplane(const std::shared_ptr<OccupancyStateInterface> &s, const std::shared_ptr<DecisionRule> &a, double delta, double learning_rate, number t)
        {

            // \beta(x,o,u) =  \beta(x,o,u) + \alpha * \delta * \xi(x,o) * a(u | o)
            auto pomdp = std::dynamic_pointer_cast<POMDPInterface>(this->getWorld()->getUnderlyingProblem());
            auto occupancy_mdp = std::dynamic_pointer_cast<OccupancyMDP>(this->getWorld());

            auto hyperplane = std::static_pointer_cast<Hyperplane>(this->getQValueFunction()->getHyperplaneAt(s, t));

            for (auto o : s->getJointHistories())
            {
                auto u = occupancy_mdp->applyDecisionRule(s, o, a, t);
                auto dr_input = occupancy_mdp->getDecisionRuleInput(o, t);
                double proba_a = a->getProbability(dr_input, u);

                for (auto x : s->getBeliefAt(o)->getStates())
                {
                    hyperplane->setValueAt(o, x, u, hyperplane->getValueAt(o, x, u) + learning_rate * delta * s->getProbability(o, x) * proba_a);
                }
            }
        }

        // std::shared_ptr<State> PWLCQUpdate::computeNewHyperplane(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<DecisionRule> &decision_rule, double delta, number t)
        // {
        //     // \beta(x,o,u) =  \beta(x,o,u) + \alpha * \delta * \xi(x,o) * a(u | o)
        //     auto pomdp = std::dynamic_pointer_cast<POMDPInterface>(this->getWorld()->getUnderlyingProblem());
        //     auto occupancy_mdp = std::dynamic_pointer_cast<OccupancyMDP>(this->getWorld());

        //     // Get old hyperplane
        //     auto old_hyperplane = std::dynamic_pointer_cast<PWLCQValueFunction::Hyperplane>(this->getQValueFunction()->getHyperplaneAt(occupancy_state, t));

        //     // Instanciate new hyperplane
        //     auto new_hyperplane = std::make_shared<PWLCQValueFunction::Hyperplane>(*old_hyperplane);
        //     new_hyperplane->state.setDefault(this->getQValueFunction()->getDefaultValue(t));

        //     // Go over all joint history for the occupancy state
        //     for (const auto &history : occupancy_state->getFullyUncompressedOccupancy()->getJointHistories())
        //     {
        //         auto &&compressed_history = occupancy_state->getCompressedJointHistory(history);
        //         auto compressed_as_state = occupancy_mdp->getDecisionRuleInput(compressed_history, t);

        //         // Select the joint action
        //         auto action = occupancy_mdp->applyDecisionRule(occupancy_state, compressed_history, decision_rule, t);

        //         // Go over all states in the belief
        //         for (const auto &state : occupancy_state->getFullyUncompressedOccupancy()->getBeliefAt(history)->getStates())
        //         {
        //             // For each hidden state with associate the value r(x,u) + discount* \sum_{x_,z_} p(x,u,z_,x_)* best_next_hyperplan(x_);
        //             double beta = this->getQValueFunction()->getBeta(old_hyperplane, state, history, action, t),
        //                    proba_o = occupancy_state->getProbability(history, state),
        //                    proba_a = decision_rule->toDecisionRule()->getProbability(compressed_as_state, action);

        //             double new_value = beta + learning_rate * delta * proba_o * proba_a;
        //             new_hyperplane->state.setValueAt(std::tuple(state, history, action), new_value);
        //         }
        //     }
        //     new_hyperplane->state.finalize();

        //     return new_hyperplane;
        // }
    }
}