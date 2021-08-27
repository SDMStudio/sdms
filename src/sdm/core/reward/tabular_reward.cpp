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

  // TabularReward::TabularReward(number num_jactions, number num_states)
  // {
  //   this->initReward(num_jactions, num_states);
  // }

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

  // const TabularReward::vector_type &TabularReward::getReward(std::shared_ptr<Action> a) const
  // {
  //   return this->rewards_[a];
  // }

  // const std::vector<TabularReward::vector_type> &TabularReward::getReward() const
  // {
  //   return this->rewards;
  // }

  // void TabularReward::setReward(std::shared_ptr<Action> a, TabularReward::vector_type v)
  // {
  //   auto r = v.min();
  //   this->min = std::min(r, this->min);

  //   r = v.max();
  //   this->max = std::max(r, this->max);

  //   this->rewards[a] = v;
  // }

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
