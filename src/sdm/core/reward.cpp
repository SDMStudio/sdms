/*=============================================================================
Copyright (C) 2016 Jilles Steeve Dibangoye
==============================================================================*/

#include <sdm/core/reward.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>

namespace sdm
{
  Reward::Reward() {}

  Reward::Reward(Reward &copy) : rewards(copy.getReward()), max(copy.getMaxReward()), min(copy.getMinReward())
  {
  }

  Reward::Reward(number num_jactions, number num_states)
  {
    this->initReward(num_jactions, num_states);
  }

  void Reward::initReward(number num_jactions, number num_states)
  {
    number a;
    for (a = 0; a < num_jactions; ++a)
    {
      this->rewards.push_back(Reward::vector_type(num_states, 0.));
    }
  }

  double Reward::getReward(number s, number a) const
  {
    return this->rewards[a][s];
  }

  const Reward::vector_type &Reward::getReward(number a) const
  {
    return this->rewards[a];
  }

  const std::vector<Reward::vector_type> &Reward::getReward() const
  {
    return this->rewards;
  }

  void Reward::setReward(number a, Reward::vector_type v)
  {
    auto r = v.min();
    this->min = std::min(r, this->min);

    r = v.max();
    this->max = std::max(r, this->max);

    this->rewards[a] = v;
  }

  void Reward::setReward(number s, number a, double r)
  {
    this->min = std::min(r, this->min);
    this->max = std::max(r, this->max);
    this->rewards[a][s] = r;
  }

  double Reward::getMaxReward() const
  {
    return this->max;
  }

  double Reward::getMinReward() const
  {
    return this->min;
  }

} // namespace sdm
