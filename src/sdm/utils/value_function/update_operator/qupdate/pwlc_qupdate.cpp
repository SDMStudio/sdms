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
            auto [state, action, reward, next_state, next_action] = this->experience_memory->sample(t)[0];
            double delta = this->delta(state, action, reward, next_state, next_action, t);
            this->updateHyperplane(state->toOccupancyState(), action->toDecisionRule(), delta, learning_rate, t);
        }

        // {

        //     auto hyperplane = std::static_pointer_cast<Hyperplane>(this->getQValueFunction()->getHyperplaneAt(s, t));
        //     auto hyperplane_ = std::static_pointer_cast<Hyperplane>(this->getQValueFunction()->getHyperplaneAt(next_state, t + 1));


        //     delta_xou = occupancy_mdp->getUnderlyingProblem()->getReward(x, u, t);


        //     if (t + 1 < occupancy_mdp->getHorizon())
        //     {
        //         for (z = 0; z < common::model->getNumObservations(); ++z)
        //         {
        //             // set next-step history h' = h + u + z
        //             auto o_ = o->expand(u, z));
        //             if (qf.find(O) == qf.end() or sPtr_->find(O) == sPtr_->end())
        //                 continue;

        //             u_ = aPtr_->getActionAt(O);
        //             for (x_ = 0; x_ < common::model->getNumStates(); ++x_)
        //             {
        //                 Qxhu += common::model->getDiscount() * common::model->getDynamics(x, u, z, x_) * qf.at(O).getQValueAt(u_, x_);
        //             }
        //         }
        //     }
        //     delat += getWorld()->getDiscount(t) *
        // }

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
    }
}