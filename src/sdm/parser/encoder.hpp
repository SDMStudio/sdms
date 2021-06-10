/*=============================================================================
Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <sdm/parser/ast.hpp>

#include <sdm/types.hpp>
#include <sdm/core/base_item.hpp>
#include <sdm/core/distribution.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/reward/tabular_reward.hpp>
#include <sdm/core/dynamics/tabular_state_dynamics.hpp>
#include <sdm/core/dynamics/tabular_observation_dynamics.hpp>
#include <sdm/world/mpomdp.hpp>

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
    // std::shared_ptr<DiscreteSpace<number>> toNumberedSpace(std::shared_ptr<DiscreteSpace> &old_sp)
    // {
    //   return std::make_shared<DiscreteSpace<number>>(old_sp.getNumItems());
    // }

    // std::shared_ptr<MultiDiscreteSpace<number>> toNumberedSpace(std::shared_ptr<MultiDiscreteSpace> &old_sp)
    // {
    //   std::vector<number> vvtmp;
    //   for (auto &dsp : old_sp.getSpaces())
    //   {
    //     vvtmp.push_back(dsp->getNumItems());
    //   }
    //   return std::make_shared<MultiDiscreteSpace<number>>(vvtmp);
    // }

    struct dpomdp_encoder : boost::static_visitor<sdm::DecPOMDP>
    {
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // DecPOMDP encoder
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////

      std::shared_ptr<sdm::DecPOMDP> operator()(dpomdp_t const &ast)
      {
        discrete_space_encoder ds_encoder;
        multi_discrete_space_encoder mds_encoder;

        // Encodes agent space
        std::shared_ptr<DiscreteSpace> agent_space = boost::apply_visitor(ds_encoder, ast.agent_param);
        std::cout << *agent_space << std::endl;

        // Encodes state space
        std::shared_ptr<DiscreteSpace> state_space = boost::apply_visitor(ds_encoder, ast.state_param);
        std::cout << *state_space << std::endl;

        // Encodes action space
        std::shared_ptr<MultiDiscreteSpace> action_space = boost::apply_visitor(mds_encoder, ast.action_param);
        std::cout << *action_space << std::endl;

        // Encodes observation space
        std::shared_ptr<MultiDiscreteSpace> obs_space = boost::apply_visitor(mds_encoder, ast.observation_param);
        std::cout << *obs_space << std::endl;

        // Encodes the reward function
        tabular_rewards_encoder rews_encoder(state_space, agent_space, action_space);
        std::shared_ptr<TabularReward> rewards = rews_encoder.encode(ast.reward_spec);

        // // Set start probabilities
        vector_encoder start_distrib_encoder(state_space->getNumItems());
        std::shared_ptr<MappedVector<number>> start_distrib__ = boost::apply_visitor(start_distrib_encoder, ast.start_param);

        auto start_distribution = std::make_shared<DiscreteDistribution<std::shared_ptr<State>>>();
        for (const auto& pair_state_proba : *start_distrib__)
        {
          start_distribution->setProbability(std::static_pointer_cast<State>(state_space->getItem(pair_state_proba.first)), pair_state_proba.second);
        }

        // // Encodes the state dynamics
        state_dynamics_encoder state_dyn_enc(state_space, agent_space, action_space);
        std::shared_ptr<TabularStateDynamics> state_dynamics = state_dyn_enc.encode(ast.transition_spec);

        // // Encodes the observation dynamics
        obs_dynamics_encoder d_encoder(state_space, agent_space, action_space, obs_space);
        std::shared_ptr<TabularObservationDynamics> obs_dynamics = d_encoder.encode(ast.observation_spec, state_dynamics);

        auto parsed_model = std::make_shared<sdm::DecPOMDP>(state_space, action_space, obs_space, rewards, state_dynamics, obs_dynamics, start_distribution, 0, ast.discount_param, (Criterion)(ast.value_param == "reward"));

#ifdef DEBUG
        std::cout << "Model Soundness=" << (parsed_model->isSound() ? "yes" : "no") << std::endl;
#ifdef VERBOSE
        std::cout << "Print model" << std::endl;
        std::cout << parsed_model << std::endl;
#endif
#endif
        return parsed_model;
      }
    };

  } // namespace ast
} // namespace sdm
