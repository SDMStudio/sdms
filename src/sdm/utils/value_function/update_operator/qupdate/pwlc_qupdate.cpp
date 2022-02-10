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

            auto hyperplane = this->getQValueFunction()->getHyperplaneAt(state, t);
            auto hyperplane_ = this->getQValueFunction()->getHyperplaneAt(next_state, t + 1);

            auto s = state->toOccupancyState();
            auto s_ = std::dynamic_pointer_cast<OccupancyState>(next_state);
            auto a = action->toDecisionRule(), a_ = next_action->toDecisionRule();

            auto occupancy_mdp = std::dynamic_pointer_cast<OccupancyMDP>(this->getWorld());
            auto pomdp = std::dynamic_pointer_cast<POMDPInterface>(this->getWorld()->getUnderlyingProblem());

            for (auto o : s->getFullyUncompressedOccupancy()->getJointHistories())
            {
                // auto &&c_o = s->getCompressedJointHistory(o);
                auto u = s->applyDR(a, s->getCompressedJointHistory(o));

                for (auto x : s->getFullyUncompressedOccupancy()->getBeliefAt(o)->getStates())
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

                                auto u_ = s_->applyDR(a_, c_o_); // a_->act(c_o_);

                                if (u_ == nullptr)
                                    continue;
                                delta_xou += getWorld()->getDiscount(t) * pomdp->getDynamics(x, u, x_, z, t) * hyperplane_->getValueAt(x_, c_o_, u_);
                            }
                        }
                    }
                    hyperplane->setValueAt(x, o, u, hyperplane->getValueAt(x, o, u) + learning_rate * (delta_xou - hyperplane->getValueAt(x, o, u)));
                }
            }
        }
    }
}