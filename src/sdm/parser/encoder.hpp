/*=============================================================================
Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <sdm/parser/ast.hpp>

#include <sdm/types.hpp>
#include <sdm/core/base_item.hpp>
#include <sdm/core/state/base_state.hpp>
#include <sdm/core/action/base_action.hpp>
#include <sdm/core/observation/base_observation.hpp>
#include <sdm/core/distribution.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/reward/tabular_reward.hpp>
#include <sdm/core/dynamics/tabular_state_dynamics.hpp>
#include <sdm/core/dynamics/tabular_observation_dynamics.hpp>
#include <sdm/world/mmdp.hpp>
#include <sdm/world/mpomdp.hpp>
#include <sdm/world/posg.hpp>

#include <sdm/parser/encoders/struct_encoders.hpp>
#include <sdm/parser/encoders/space_encoders.hpp>
#include <sdm/parser/encoders/item_encoders.hpp>
#include <sdm/parser/encoders/reward_encoders.hpp>
#include <sdm/parser/encoders/dynamics_encoders.hpp>

///////////////////////////////////////////////////////////////////////////////
//  AST processing
///////////////////////////////////////////////////////////////////////////////
namespace sdm
{
  namespace ast
  {

    struct mdp_encoder : boost::static_visitor<sdm::MDP>
    {
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // MMDP encoder
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////

      template <typename AST>
      std::shared_ptr<sdm::MDP> operator()(AST const &ast)
      {
        discrete_space_encoder<StringState> ds_state_encoder;
        multi_discrete_space_encoder<StringAction> mds_action_encoder;

        // Encodes agent space
        auto agent_id = std::make_shared<StringItem>("0");
        auto agent_space = std::make_shared<DiscreteItemSpace>(std::vector<std::shared_ptr<StringItem>>{agent_id});

        // Encodes state space
        auto state_space = boost::apply_visitor(ds_state_encoder, ast.state_param);

        // Encodes action space
        auto action_space = boost::apply_visitor(mds_action_encoder, ast.action_param);

        // Encodes the reward function
        tabular_rewards_encoder rews_encoder(state_space, agent_space, action_space);
        std::shared_ptr<CooperativeRewardModel> rewards = rews_encoder.encode(ast.reward_spec);

        // // Set start probabilities
        vector_encoder start_distrib_encoder(state_space->getNumItems());
        std::shared_ptr<MappedVector<number>> start_distrib__ = boost::apply_visitor(start_distrib_encoder, ast.start_param);

        auto start_distribution = std::make_shared<DiscreteDistribution<std::shared_ptr<State>>>();
        for (const auto &pair_state_proba : *start_distrib__)
        {
          start_distribution->setProbability(state_space->getItem(pair_state_proba.first)->toState(), pair_state_proba.second);
        }

        // // Encodes the state dynamics
        state_dynamics_encoder state_dyn_enc(state_space, agent_space, action_space);
        std::shared_ptr<TabularStateDynamics> state_dynamics = state_dyn_enc.encode(ast.transition_spec);

        auto parsed_model = std::make_shared<sdm::MDP>(state_space, action_space, rewards, state_dynamics, start_distribution, 0, ast.discount_param, (Criterion)(ast.value_param == "reward"));

        return parsed_model;
      }
    };

    struct mmdp_encoder : boost::static_visitor<sdm::MMDP>
    {
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // MMDP encoder
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////

      template <typename AST>
      std::shared_ptr<sdm::MMDP> operator()(AST const &ast, bool is_multi_agent = true)
      {
        discrete_space_encoder<StringItem> agent_encoder;
        discrete_space_encoder<StringState> ds_state_encoder;
        multi_discrete_space_encoder<StringAction> mds_action_encoder;

        std::shared_ptr<DiscreteItemSpace> agent_space;
        // Encodes agent space
        if (is_multi_agent)
        {
          agent_space = boost::apply_visitor(agent_encoder, ast.agent_param);
        }
        else
        {
          // Encodes agent space
          auto agent_id = std::make_shared<StringItem>("0");
          agent_space = std::make_shared<DiscreteItemSpace>(std::vector<std::shared_ptr<StringItem>>{agent_id});
        }

        // Encodes state space
        auto state_space = boost::apply_visitor(ds_state_encoder, ast.state_param);

        // Encodes action space
        auto action_space = boost::apply_visitor(mds_action_encoder, ast.action_param);

        // Encodes the reward function
        tabular_rewards_encoder rews_encoder(state_space, agent_space, action_space);
        std::shared_ptr<CooperativeRewardModel> rewards = rews_encoder.encode(ast.reward_spec);

        // // Set start probabilities
        vector_encoder start_distrib_encoder(state_space->getNumItems());
        std::shared_ptr<MappedVector<number>> start_distrib__ = boost::apply_visitor(start_distrib_encoder, ast.start_param);

        auto start_distribution = std::make_shared<DiscreteDistribution<std::shared_ptr<State>>>();
        for (const auto &pair_state_proba : *start_distrib__)
        {
          start_distribution->setProbability(state_space->getItem(pair_state_proba.first)->toState(), pair_state_proba.second);
        }

        // // Encodes the state dynamics
        state_dynamics_encoder state_dyn_enc(state_space, agent_space, action_space);
        std::shared_ptr<TabularStateDynamics> state_dynamics = state_dyn_enc.encode(ast.transition_spec);

        auto parsed_model = std::make_shared<sdm::MMDP>(state_space, action_space, rewards, state_dynamics, start_distribution, 0, ast.discount_param, (Criterion)(ast.value_param == "reward"));

        return parsed_model;
      }
    };

    struct pomdp_encoder : boost::static_visitor<sdm::POMDP>
    {
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // MPOMDP encoder
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////

      template <typename AST>
      std::shared_ptr<sdm::POMDP> operator()(AST const &ast)
      {
        discrete_space_encoder<StringState> ds_state_encoder;
        multi_discrete_space_encoder<StringAction> mds_action_encoder;
        multi_discrete_space_encoder<StringObservation> mds_observation_encoder;

        // Encodes agent space
        auto agent_id = std::make_shared<StringItem>("0");
        auto agent_space = std::make_shared<DiscreteItemSpace>(std::vector<std::shared_ptr<StringItem>>{agent_id});

        // Encodes state space
        auto state_space = boost::apply_visitor(ds_state_encoder, ast.state_param);

        // Encodes action space
        auto action_space = boost::apply_visitor(mds_action_encoder, ast.action_param);

        // Encodes observation space
        auto obs_space = boost::apply_visitor(mds_observation_encoder, ast.observation_param);

        // Encodes the reward function
        tabular_rewards_encoder rews_encoder(state_space, agent_space, action_space);
        std::shared_ptr<CooperativeRewardModel> rewards = rews_encoder.encode(ast.reward_spec);

        // // Set start probabilities
        vector_encoder start_distrib_encoder(state_space->getNumItems());
        std::shared_ptr<MappedVector<number>> start_distrib__ = boost::apply_visitor(start_distrib_encoder, ast.start_param);

        auto start_distribution = std::make_shared<DiscreteDistribution<std::shared_ptr<State>>>();
        for (const auto &pair_state_proba : *start_distrib__)
        {
          start_distribution->setProbability(state_space->getItem(pair_state_proba.first)->toState(), pair_state_proba.second);
        }

        // // Encodes the state dynamics
        state_dynamics_encoder state_dyn_enc(state_space, agent_space, action_space);
        std::shared_ptr<TabularStateDynamics> state_dynamics = state_dyn_enc.encode(ast.transition_spec);

        // // Encodes the observation dynamics
        obs_dynamics_encoder d_encoder(state_space, agent_space, action_space, obs_space);
        std::shared_ptr<TabularObservationDynamics> obs_dynamics = d_encoder.encode(ast.observation_spec, state_dynamics);

        auto parsed_model = std::make_shared<sdm::POMDP>(state_space, action_space, obs_space, rewards, state_dynamics, obs_dynamics, start_distribution, 0, ast.discount_param, (Criterion)(ast.value_param == "reward"));

        return parsed_model;
      }
    };

    struct dpomdp_encoder : boost::static_visitor<sdm::DecPOMDP>
    {
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // MPOMDP encoder
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////

      template <typename AST>
      std::shared_ptr<sdm::DecPOMDP> operator()(AST const &ast, bool is_multi_agent = true)
      {
        discrete_space_encoder<StringItem> agent_encoder;
        discrete_space_encoder<StringState> ds_state_encoder;
        multi_discrete_space_encoder<StringAction> mds_action_encoder;
        multi_discrete_space_encoder<StringObservation> mds_observation_encoder;

        // Encodes agent space
        std::shared_ptr<DiscreteItemSpace> agent_space;
        // Encodes agent space
        if (is_multi_agent)
        {
          agent_space = boost::apply_visitor(agent_encoder, ast.agent_param);
        }
        else
        {
          // Encodes agent space
          auto agent_id = std::make_shared<StringItem>("0");
          agent_space = std::make_shared<DiscreteItemSpace>(std::vector<std::shared_ptr<StringItem>>{agent_id});
        }

        // Encodes state space
        auto state_space = boost::apply_visitor(ds_state_encoder, ast.state_param);

        // Encodes action space
        auto action_space = boost::apply_visitor(mds_action_encoder, ast.action_param);

        // Encodes observation space
        auto obs_space = boost::apply_visitor(mds_observation_encoder, ast.observation_param);

        // Encodes the reward function
        tabular_rewards_encoder rews_encoder(state_space, agent_space, action_space);
        std::shared_ptr<CooperativeRewardModel> rewards = rews_encoder.encode(ast.reward_spec);

        // // Set start probabilities
        vector_encoder start_distrib_encoder(state_space->getNumItems());
        std::shared_ptr<MappedVector<number>> start_distrib__ = boost::apply_visitor(start_distrib_encoder, ast.start_param);

        auto start_distribution = std::make_shared<DiscreteDistribution<std::shared_ptr<State>>>();
        for (const auto &pair_state_proba : *start_distrib__)
        {
          start_distribution->setProbability(state_space->getItem(pair_state_proba.first), pair_state_proba.second);
        }

        // // Encodes the state dynamics
        state_dynamics_encoder state_dyn_enc(state_space, agent_space, action_space);
        std::shared_ptr<TabularStateDynamics> state_dynamics = state_dyn_enc.encode(ast.transition_spec);

        // // Encodes the observation dynamics
        obs_dynamics_encoder d_encoder(state_space, agent_space, action_space, obs_space);
        std::shared_ptr<TabularObservationDynamics> obs_dynamics = d_encoder.encode(ast.observation_spec, state_dynamics);

        auto parsed_model = std::make_shared<sdm::DecPOMDP>(state_space, action_space, obs_space, rewards, state_dynamics, obs_dynamics, start_distribution, 0, ast.discount_param, (Criterion)(ast.value_param == "reward"));

        return parsed_model;
      }
    };

    struct posg_encoder : boost::static_visitor<sdm::POSG>
    {
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // POSG encoder
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////

      std::shared_ptr<sdm::POSG> operator()(posg_t const &ast)
      {
        discrete_space_encoder<StringItem> agent_encoder;
        discrete_space_encoder<StringState> ds_state_encoder;
        multi_discrete_space_encoder<StringAction> mds_action_encoder;
        multi_discrete_space_encoder<StringObservation> mds_observation_encoder;

        // Encodes agent space
        auto agent_space = boost::apply_visitor(agent_encoder, ast.agent_param);

        // Encodes state space
        auto state_space = boost::apply_visitor(ds_state_encoder, ast.state_param);

        // Encodes action space
        auto action_space = boost::apply_visitor(mds_action_encoder, ast.action_param);

        // Encodes observation space
        auto obs_space = boost::apply_visitor(mds_observation_encoder, ast.observation_param);

        // Encodes the reward function
        multi_tabular_rewards_encoder rews_encoder(state_space, agent_space, action_space);
        std::shared_ptr<CompetitiveRewardModel> rewards = rews_encoder.encode(ast.multi_reward_spec);

        // // Set start probabilities
        vector_encoder start_distrib_encoder(state_space->getNumItems());
        std::shared_ptr<MappedVector<number>> start_distrib__ = boost::apply_visitor(start_distrib_encoder, ast.start_param);

        auto start_distribution = std::make_shared<DiscreteDistribution<std::shared_ptr<State>>>();
        for (const auto &pair_state_proba : *start_distrib__)
        {
          start_distribution->setProbability(state_space->getItem(pair_state_proba.first)->toState(), pair_state_proba.second);
        }

        // // Encodes the state dynamics
        state_dynamics_encoder state_dyn_enc(state_space, agent_space, action_space);
        std::shared_ptr<TabularStateDynamics> state_dynamics = state_dyn_enc.encode(ast.transition_spec);

        // // Encodes the observation dynamics
        obs_dynamics_encoder d_encoder(state_space, agent_space, action_space, obs_space);
        std::shared_ptr<TabularObservationDynamics> obs_dynamics = d_encoder.encode(ast.observation_spec, state_dynamics);

        auto parsed_model = std::make_shared<sdm::POSG>(state_space, action_space, obs_space, rewards, state_dynamics, obs_dynamics, start_distribution, 0, ast.discount_param, (Criterion)(ast.value_param == "reward"));

        return parsed_model;
      }
    };

  } // namespace ast
} // namespace sdm
