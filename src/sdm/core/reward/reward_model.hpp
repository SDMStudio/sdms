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
  class RewardModel
  {
  public:
    virtual double getMinReward(number agent_id, number t) const = 0;
    virtual double getMaxReward(number agent_id, number t) const = 0;
    virtual double getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number agent_id, number t) const = 0;
    
  };

  /**
   * @brief This class provides a common interface for every models of reward with tabular representation.
   */
  class TabularRewardModel : public RewardModel
  {
  public:
    virtual double getMinReward(number agent_id, number t) const = 0;
    virtual double getMaxReward(number agent_id, number t) const = 0;
    virtual double getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number agent_id, number t) const = 0;
    virtual void setReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number agent_id, double reward, number t) = 0;
    
  };
}
