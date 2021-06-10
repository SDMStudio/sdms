#pragma once

#include <sdm/parser/ast.hpp>

#include <sdm/core/item.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/dynamics/tabular_state_dynamics.hpp>
#include <sdm/core/dynamics/tabular_observation_dynamics.hpp>

#include <sdm/parser/encoders/item_encoders.hpp>
#include <sdm/parser/encoders/struct_encoders.hpp>

namespace sdm
{
    namespace ast
    {

        /**
         * @brief encodes state transition dynamics (i.e. TabularStateDynamics class)
         * 
         */
        class state_transition_encoder : boost::static_visitor<>
        {
        protected:
            std::shared_ptr<DiscreteSpace> state_space_;
            std::shared_ptr<DiscreteSpace> agent_space_;
            std::shared_ptr<MultiDiscreteSpace> action_space_;

            std::shared_ptr<TabularStateDynamics> state_dynamics_;

        public:
            state_transition_encoder(const std::shared_ptr<DiscreteSpace> &st_space,
                                     const std::shared_ptr<DiscreteSpace> &ag_space,
                                     const std::shared_ptr<MultiDiscreteSpace> &act_space,
                                     const std::shared_ptr<TabularStateDynamics> &state_dynamics);

            void operator()(const transition_entry_1_t &t1);
            void operator()(const transition_entry_2_t &t2);
            void operator()(const transition_entry_3_t &t3);
        };

        /**
         * @brief encodes state transition dynamics (i.e. TabularStateDynamics class)
         * 
         */
        class state_dynamics_encoder
        {
        protected:
            std::shared_ptr<DiscreteSpace> state_space_;
            std::shared_ptr<DiscreteSpace> agent_space_;
            std::shared_ptr<MultiDiscreteSpace> action_space_;

        public:
            state_dynamics_encoder(const std::shared_ptr<DiscreteSpace> &state_space, const std::shared_ptr<DiscreteSpace> &agent_space, const std::shared_ptr<MultiDiscreteSpace> &action_space);
            std::shared_ptr<TabularStateDynamics> encode(const transition_t &transits);
        };

        /**
         * @brief encodes the input into an observation dynamic class
         * 
         */
        struct observation_transition_encoder : boost::static_visitor<>
        {
            std::shared_ptr<DiscreteSpace> state_space_;
            std::shared_ptr<DiscreteSpace> agent_space_;
            std::shared_ptr<MultiDiscreteSpace> action_space_;
            std::shared_ptr<MultiDiscreteSpace> obs_space_;

            std::shared_ptr<TabularObservationDynamics> obs_dynamics_;

            observation_transition_encoder(const std::shared_ptr<DiscreteSpace> &st_space,
                                           const std::shared_ptr<DiscreteSpace> &ag_space,
                                           const std::shared_ptr<MultiDiscreteSpace> &act_space,
                                           const std::shared_ptr<MultiDiscreteSpace> &obs_space,
                                           const std::shared_ptr<TabularObservationDynamics> &dynamics);

            void operator()(const observation_entry_1_t &z1);
            void operator()(const observation_entry_3_t &z3);
            void operator()(const observation_entry_2_t &z2);
        };

        class obs_dynamics_encoder
        {
        protected:
            std::shared_ptr<DiscreteSpace> state_space_;
            std::shared_ptr<DiscreteSpace> agent_space_;
            std::shared_ptr<MultiDiscreteSpace> action_space_;
            std::shared_ptr<MultiDiscreteSpace> obs_space_;

        public:
            obs_dynamics_encoder(const std::shared_ptr<DiscreteSpace> &state_space, const std::shared_ptr<DiscreteSpace> &agent_space, const std::shared_ptr<MultiDiscreteSpace> &action_space, const std::shared_ptr<MultiDiscreteSpace> &obs_space);
            std::shared_ptr<TabularObservationDynamics> encode(const observation_t &observs, std::shared_ptr<StateDynamicsInterface> state_dynamics);
        };

    } // namespace ast

} // namespace sdm
