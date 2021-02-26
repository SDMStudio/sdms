/**
 * @file decision_process.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief 
 * @version 1.0
 * @date 02/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <vector>

#include <sdm/types.hpp>
#include <sdm/public/world.hpp>
#include <sdm/world/base/decision_process_base.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/state_dynamics.hpp>
#include <sdm/core/reward.hpp>

namespace sdm
{

        template <typename TStateSpace, typename TActionSpace, typename TStateDynamics, typename TReward, typename TDistrib>
        class DecisionProcess : public DecisionProcessBase<TStateSpace, TActionSpace, TDistrib>, public World
        {
        public:
                DecisionProcess();
                DecisionProcess(std::shared_ptr<TStateSpace> state_sp, std::shared_ptr<TActionSpace> action_sp);
                DecisionProcess(std::shared_ptr<TStateSpace> state_sp, std::shared_ptr<TActionSpace> action_sp, TDistrib);
                DecisionProcess(std::shared_ptr<TStateSpace> state_sp, std::shared_ptr<TActionSpace> action_sp, std::shared_ptr<TStateDynamics>, std::shared_ptr<TReward>, TDistrib start_distrib, number planning_horizon = 0, double discount = 0.9, Criterion criterion = Criterion::REW_MAX);

                /**
                 * \brief Get the state dynamics
                 */
                std::shared_ptr<TStateDynamics> getStateDynamics() const;

                /**
                 * \brief Set the state dynamics
                 */
                void setStateDynamics(std::shared_ptr<TStateDynamics> state_dyn);

                /**
                 * \brief Get the reward function
                 */
                std::shared_ptr<TReward> getReward() const;

                /**
                 * \brief Set the reward function
                 */
                void setReward(std::shared_ptr<TReward> reward_function);

                // /**
                //  * \brief Get transition probability from joint action
                //  */
                // // double getTransitionProba(state_type cstate, action_type jaction, state_type state);

                // TDistrib getProbaNextState(state_type cstate)
                // {
                //         // std::vector<number> vect;
                //         // for (double a : this->getActionSpace().getAll())
                //         // {
                //         //         auto probas = this->getProbaNextState(cstate, a).probabilities();
                //         //         vect.insert(vect.end(), probas.begin(), probas.end());
                //         // }
                //         // return std::discrete_distribution<number>(vect.begin(), vect.end());
                // }

                // /**
                //  * \brief Transit to next state given a joint action
                //  */
                // void nextState();

                // TDistrib getProbaNextState(state_type cstate, action_type caction)
                // {
                //         return this->getStateDynamics().getProbaNextState(cstate, caction);
                // }

                // /**
                //  * \brief Transit to next state given a joint action
                //  */
                // void nextState(action_type jaction);

                // TReward &getReward();

                // /**
                //  * \brief Get transition probability from joint action (as a single one) for all agents
                //  */
                // double getReward(state_type state, action_type jaction);

                // /**
                //  * \fn std::vector<double> getReward(number state, std::vector<number> jaction);
                //  * \brief Get cost from joint action for all agents
                //  */
                // std::vector<double> getCost(state_type state, action_type jaction);

        protected:
                /**
                *  @brief Space of agents (contain number of agents and their names).
                */
                number num_agents_;

                /**
                 * @brief State dynamics.
                 */
                std::shared_ptr<TStateDynamics> state_dynamics_;

                /**
                 * @brief Reward functions.
                 */
                std::shared_ptr<TReward> reward_function_;
        };

        using DiscreteMDP = DecisionProcess<DiscreteSpace, DiscreteSpace, StateDynamics, Reward, std::discrete_distribution<number>>;
        using DiscreteDecMDP = DecisionProcess<DiscreteSpace, MultiDiscreteSpace, StateDynamics, Reward, std::discrete_distribution<number>>;
        using DiscreteSG = DecisionProcess<DiscreteSpace, MultiDiscreteSpace, StateDynamics, std::vector<Reward>, std::discrete_distribution<number>>;

        // template <>
        // class DecisionProcess<MultiDiscreteSpace, MultiDiscreteSpace, StateDynamics, std::discrete_distribution<number>>
        // {
        //         double getTransitionProba(number cstate, number jaction, number nstate)
        //         {
        //                 return this->s_dynamics_.getTransitionProbability(cstate, jaction, nstate);
        //         }

        //         double getTransitionProba(number cstate, Joint<number> jaction, number nstate)
        //         {
        //                 return this->getTransitionProba(cstate, this->getActionSpace().joint2single(jaction), nstate);
        //         }

        //         double getReward(number cstate, number jaction)
        //         {
        //                 return this->reward.getTransitionProbability(cstate, jaction, nstate);
        //         }

        //         double getReward(number cstate, Joint<number> jaction)
        //         {
        //                 return this->getTransitionProba(cstate, this->getActionSpace().joint2single(jaction), nstate);
        //         }
        // };

        // template <>
        // class DecisionProcess<DiscreteSpace, DiscreteSpace, StateDynamics, std::discrete_distribution<number>>
        // {
        //         TDistrib getProbaNextState(state_type cstate)
        //         {
        //                 // std::vector<number> vect;
        //                 // for (double a : this->getActionSpace().getAll())
        //                 // {
        //                 //         auto probas = this->getProbaNextState(cstate, a).probabilities();
        //                 //         vect.insert(vect.end(), probas.begin(), probas.end());
        //                 // }
        //                 // return std::discrete_distribution<number>(vect.begin(), vect.end());
        //         }

        //         /**
        //          * \brief Transit to next state given a joint action
        //          */
        //         void nextState();

        //         TDistrib getProbaNextState(state_type cstate, action_type caction)
        //         {
        //                 return this->getStateDynamics().getProbaNextState(cstate, caction);
        //         }

        //         /**
        //          * \brief Transit to next state given a joint action
        //          */
        //         void nextState(action_type jaction);

        //         double getTransitionProba(number cstate, number action, number nstate)
        //         {
        //                 return this->s_dynamics_.getTransitionProbability(cstate, action, nstate);
        //         }
        // };

        // template <typename TStateSpace, typename TActionSpace, typename TDistrib>
        // using SG = DecisionProcess<TStateSpace, TActionSpace, TDistrib>;

        // using DiscreteMDP = DecisionProcess<DiscreteSpace, DiscreteSpace, std::discrete_distribution<number>>;
        // using DiscreteSG = DecisionProcess<MultiDiscreteSpace, MultiDiscreteSpace, std::discrete_distribution<number>>;
} // namespace sdm
#include <sdm/world/decision_process.tpp>
