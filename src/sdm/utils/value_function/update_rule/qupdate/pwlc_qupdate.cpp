#include <sdm/utils/value_function/update_rule/qupdate/pwlc_qupdate.hpp>
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
            std::shared_ptr<ValueFunctionInterface> target_q_value) : PWLCQUpdateRule(experience_memory, q_value, target_q_value)
        {
        }

        void PWLCQUpdate::update(double learning_rate, number t)
        {
            auto [state, action, reward, next_state, next_action] = this->experience_memory->sample(t)[0]; // 1.92

            auto hyperplane = this->getQValueFunction()->getHyperplaneAt(state, t);           // 1.15
            auto hyperplane_ = this->getQValueFunction()->getHyperplaneAt(next_state, t + 1); // 1.15

            auto s = state->toOccupancyState();
            auto s_ = std::dynamic_pointer_cast<OccupancyState>(next_state);
            auto a = action->toDecisionRule(), a_ = next_action->toDecisionRule();

            auto pomdp = std::dynamic_pointer_cast<POMDPInterface>(this->getWorld()->getUnderlyingProblem());

            for (auto o : s->getFullyUncompressedOccupancy()->getJointHistories())
            {
                // auto &&c_o = s->getCompressedJointHistory(o);
                auto u = s->applyDR(a, s->getCompressedJointHistory(o));

                for (auto x : s->getFullyUncompressedOccupancy()->getBeliefAt(o)->getStates())
                {
                    double delta_xou = pomdp->getReward(x, u, t);

                    if ((pomdp->getHorizon() == 0) || (t + 1 < pomdp->getHorizon()))
                    {
                        for (const auto &x_ : pomdp->getReachableStates(x, u, t))
                        {
                            for (const auto &z : *pomdp->getObservationSpace(t)) // pomdp->getReachableObservations(x, u, x_, t)) // 0.86
                            {
                                // set next-step history h' = h + u + z
                                auto o_ = o->expand(std::static_pointer_cast<JointObservation>(z)); // 5.2
                                auto &&c_o_ = s_->getCompressedJointHistory(o_);                    // 5.8

                                if (s_->getProbability(c_o_) == 0) // 0.54
                                    continue;

                                auto u_ = s_->applyDR(a_, c_o_); // a_->act(c_o_); // 8.39

                                if (u_ == nullptr)
                                    continue;
                                delta_xou += getWorld()->getDiscount(t) * pomdp->getDynamics(x, u, x_, z->toObservation(), t) * hyperplane_->getValueAt(x_, c_o_, u_);
                            }
                        }
                    }
                    hyperplane->setValueAt(x, o, u, hyperplane->getValueAt(x, o, u) + learning_rate * (delta_xou - hyperplane->getValueAt(x, o, u)));
                }
            }
        }
    }
}