#include <sdm/utils/value_function/update_operator/qupdate/serial_pwlc_qupdate.hpp>
#include <sdm/core/state/serial_state.hpp>
#include <sdm/core/state/belief_state.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/state/occupancy_state_serial.hpp>
#include <sdm/world/base/pomdp_interface.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/utils/value_function/pwlc_value_function_interface.hpp>
#include <sdm/utils/value_function/qfunction/pwlc_qvalue_function.hpp>

namespace sdm
{
    namespace update
    {
        SerialPWLCQUpdate::SerialPWLCQUpdate(
            std::shared_ptr<ExperienceMemory> experience_memory,
            std::shared_ptr<ValueFunctionInterface> q_value,
            std::shared_ptr<ValueFunctionInterface> target_q_value) : PWLCQUpdate(experience_memory, q_value, target_q_value)
        {
        }

        void SerialPWLCQUpdate::update(double learning_rate, number t)
        {
            auto [state, action, reward, next_state, next_action] = this->experience_memory->sample(t)[0]; // 1.92

            auto hyperplane = this->getQValueFunction()->getHyperplaneAt(state, t);           // 1.15
            auto hyperplane_ = this->getQValueFunction()->getHyperplaneAt(next_state, t + 1); // 1.15

            auto s = std::dynamic_pointer_cast<OccupancyStateSerial>(state);
            auto s_ = std::dynamic_pointer_cast<OccupancyStateSerial>(next_state);
            auto a = action->toDecisionRule(), a_ = next_action->toDecisionRule();

            auto pomdp = std::dynamic_pointer_cast<POMDPInterface>(this->getWorld()->getUnderlyingProblem());

            for (auto o : s->getFullyUncompressedOccupancy()->getJointHistories())
            {
                // auto &&c_o = s->getCompressedJointHistory(o);
                auto indiv_u = s->applyIndivDR(a, s->getCompressedJointHistory(o));

                for (auto x : s->getFullyUncompressedOccupancy()->getBeliefAt(o)->getStates())
                {
                    if ((pomdp->getHorizon() == 0) || (t + 1 < pomdp->getHorizon()))
                    {
                        if (s->isLastAgent(t))
                        {
                            auto joint_u = s->getFullAction(pomdp, s->actions[o], indiv_u, t);
                            double delta_xou = pomdp->getReward(x, joint_u, t);
                            std::cout << "x=" << x->str() << std::endl;
                            std::cout << "o=" << o->short_str() << std::endl;
                            std::cout << "joint_u=" << joint_u->str() << std::endl;
                            for (const auto &x_ : pomdp->getReachableStates(x, joint_u, t))
                            {
                                for (const auto &z : pomdp->getReachableObservations(x, joint_u, x_, t)) // 0.86
                                {
                                    std::cout << "x=" << x->str() << "  z=" << z->str() << std::endl;
                                    // set next-step history h' = h + u + z
                                    auto o_ = o->expand(std::static_pointer_cast<JointObservation>(z)); // 5.2
                                    auto &&c_o_ = s_->getCompressedJointHistory(o_);                    // 5.8

                                    if (s_->getProbability(c_o_) == 0) // 0.54
                                        continue;

                                    auto u_ = s_->applyIndivDR(a_, c_o_); // 8.39

                                    std::cout << "u_=" << u_->str() << std::endl;
                                    if (u_ == nullptr)
                                        continue;
                                    delta_xou += getWorld()->getDiscount(t) * pomdp->getDynamics(x, joint_u, x_, z, t) * hyperplane_->getValueAt(x_, c_o_, u_);
                                }
                            }
                            std::cout << "delta_xou=" << delta_xou << std::endl;
                            hyperplane->setValueAt(x, o, indiv_u, hyperplane->getValueAt(x, o, indiv_u) + learning_rate * (delta_xou - hyperplane->getValueAt(x, o, indiv_u)));
                        }
                        else
                        {
                            auto u_list = s->actions[o];
                            u_list.push_back(indiv_u);
                            auto u_ = s_->applyIndivDR(a_, o);
                            double delta_xou = hyperplane_->getValueAt(x, o, u_);
                            std::cout << "delta_xou(2)=" << delta_xou << std::endl;
                            hyperplane->setValueAt(x, o, indiv_u, hyperplane->getValueAt(x, o, indiv_u) + learning_rate * (delta_xou - hyperplane->getValueAt(x, o, indiv_u)));
                        }
                    }
                }
            }
        }
    }
}