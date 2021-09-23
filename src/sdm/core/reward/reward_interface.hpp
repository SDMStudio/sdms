/**
 * @file reward.hpp
 * @author Jilles S. Dibangoye
 * @brief This file contains the implementation of the Reward model.
 * @version 1.0
 * @date 09/03/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/types.hpp>

namespace sdm
{
    /**
     * @brief This class provides a common interface for every models of reward.
     * 
     * A model of reward is well defined if it implements this interface. A reward model is required to define a MDP (see file MDP.hpp) and related problems.
     * Basically, there is two types of reward models : the tabular model (tabular_reward.hpp) and the function based model (function_reward.hpp).
     */
  class RewardInterface
  {
  public:
    virtual double getMinReward(number t) const = 0;
    virtual double getMaxReward(number t) const = 0;
    virtual double getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const = 0;
  };


  // class BaseMultiReward
  // {
  // public:
  //   virtual double getMinReward(number t, number agent_id) const = 0;
  //   virtual double getMaxReward(number t, number agent_id) const = 0;
  //   virtual double getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t, number agent_id) const = 0;
  // };
}
