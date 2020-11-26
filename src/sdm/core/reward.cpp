/*=============================================================================
Copyright (C) 2016 Jilles Steeve Dibangoye
==============================================================================*/

#include <sdm/core/reward.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>

namespace sdm
{
  Reward::Reward(){}

  Reward::Reward(number num_jactions, number num_states)
  {
    this->initReward(num_jactions, num_states);
  }

  void Reward::initReward(number num_jactions, number num_states)
  {
    number a;
    for (a = 0; a < num_jactions; ++a)
    {
      auto v = Vector(num_states);
      v.init(0.0);
      this->rewards.push_back(v);
    }
  }

  double Reward::getReward(number s, number a) const
  {
    return this->rewards[a][s];
  }

  const Vector &Reward::getReward(number a) const
  {
    return this->rewards[a];
  }

  const std::vector<Vector> &Reward::getReward() const
  {
    return this->rewards;
  }

  void Reward::setReward(number a, const Vector &v)
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
