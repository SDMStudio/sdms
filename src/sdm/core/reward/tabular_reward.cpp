/*=============================================================================
Copyright (C) 2016 Jilles Steeve Dibangoye
==============================================================================*/

#include <sdm/core/reward/tabular_reward.hpp>

namespace sdm
{
  TabularReward::TabularReward() {}
  
  TabularReward::~TabularReward() {}

  TabularReward::TabularReward(const TabularReward &copy) : rewards_(copy.rewards_), max(copy.max), min(copy.min)
  {
  }

  void TabularReward::initReward(number, number)
  {
    // number a;
    // for (a = 0; a < num_action; ++a)
    // {
    //   this->rewards.push_back(TabularReward::vector_type(num_states, 0.));
    // }
  }

  double TabularReward::getReward(const std::shared_ptr<State> &s, const std::shared_ptr<Action> &a, number) const
  {
    return this->rewards_.getValueAt(s, a);
  }

  void TabularReward::setReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, double reward, number)
  {
    this->min = std::min(reward, this->min);
    this->max = std::max(reward, this->max);
    return this->rewards_.setValueAt(state, action, reward);
  }

  double TabularReward::getMaxReward(number) const
  {
    return this->max;
  }

  double TabularReward::getMinReward(number) const
  {
    return this->min;
  }

} // namespace sdm
