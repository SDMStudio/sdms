/*=============================================================================
Copyright (C) 2016 Jilles Steeve Dibangoye
==============================================================================*/

#include <sdm/core/reward/tabular_reward.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>

namespace sdm
{
  TabularReward::Reward() {}

  TabularReward::Reward(Reward &copy) : rewards(copy.getReward()), max(copy.getMaxReward()), min(copy.getMinReward())
  {
  }

  TabularReward::Reward(number num_jactions, number num_states)
  {
    this->initReward(num_jactions, num_states);
  }

  void TabularReward::initReward(number num_jactions, number num_states)
  {
    number a;
    for (a = 0; a < num_jactions; ++a)
    {
      this->rewards.push_back(TabularReward::vector_type(num_states, 0.));
    }
  }

  double TabularReward::getReward(std::shared_ptr<State> s, std::shared_ptr<Action> a) const
  {
    return this->rewards[a][s];
  }

  const TabularReward::vector_type &TabularReward::getReward(std::shared_ptr<Action> a) const
  {
    return this->rewards[a];
  }

  const std::vector<TabularReward::vector_type> &TabularReward::getReward() const
  {
    return this->rewards;
  }

  void TabularReward::setReward(std::shared_ptr<Action> a, TabularReward::vector_type v)
  {
    auto r = v.min();
    this->min = std::min(r, this->min);

    r = v.max();
    this->max = std::max(r, this->max);

    this->rewards[a] = v;
  }

  void TabularReward::setReward(std::shared_ptr<State> s, std::shared_ptr<Action> a, double r)
  {
    this->min = std::min(r, this->min);
    this->max = std::max(r, this->max);
    this->rewards[a][s] = r;
  }

  double TabularReward::getMaxReward() const
  {
    return this->max;
  }

  double TabularReward::getMinReward() const
  {
    return this->min;
  }

} // namespace sdm
