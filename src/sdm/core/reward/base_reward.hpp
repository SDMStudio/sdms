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
  class BaseReward
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
