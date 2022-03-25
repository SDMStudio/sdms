/*=============================================================================
Copyright (C) 2016 Jilles Steeve Dibangoye
==============================================================================*/

#include <sdm/core/reward/tabular_reward.hpp>

namespace sdm
{
  CooperativeRewardModel::CooperativeRewardModel() {}

  CooperativeRewardModel::~CooperativeRewardModel() {}

  CooperativeRewardModel::CooperativeRewardModel(const CooperativeRewardModel &copy) : rewards_(copy.rewards_), max(copy.max), min(copy.min)
  {
  }

  void CooperativeRewardModel::initReward(number, number)
  {
  }

  double CooperativeRewardModel::getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number, number) const
  {
    return this->rewards_.getValueAt(state, action);
  }

  void CooperativeRewardModel::setReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number, double reward, number)
  {
    this->min = std::min(reward, this->min);
    this->max = std::max(reward, this->max);
    this->rewards_.setValueAt(state, action, reward);
  }

  double CooperativeRewardModel::getMaxReward(number, number) const
  {
    return this->max;
  }

  double CooperativeRewardModel::getMinReward(number, number) const
  {
    return this->min;
  }

  CompetitiveRewardModel::CompetitiveRewardModel() {}

  CompetitiveRewardModel::~CompetitiveRewardModel() {}

  CompetitiveRewardModel::CompetitiveRewardModel(const CompetitiveRewardModel &copy) : rewards_(copy.rewards_), max(copy.max), min(copy.min)
  {
  }

  void CompetitiveRewardModel::initReward(number, number)
  {
  }

  double CompetitiveRewardModel::getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number agent_id, number) const
  {
    return this->rewards_.at(state).at(action).at(agent_id);
  }

  void CompetitiveRewardModel::setReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, std::vector<double> rewards, number)
  {
    if (min.size() == 0)
    {
      this->min = std::vector<double>(rewards.size(), -std::numeric_limits<double>::max());
      this->max = std::vector<double>(rewards.size(), std::numeric_limits<double>::max());
    }
    for (int ag = 0; ag < rewards.size(); ag++)
    {
      this->min[ag] = std::min(rewards[ag], this->min[ag]);
      this->max[ag] = std::max(rewards[ag], this->max[ag]);
    }
    this->rewards_[state][action] = rewards;
  }

  void CompetitiveRewardModel::setReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number agent_id, double reward, number)
  {
    this->min[agent_id] = std::min(reward, this->min[agent_id]);
    this->max[agent_id] = std::max(reward, this->max[agent_id]);
    this->rewards_[state][action][agent_id] = reward;
  }

  double CompetitiveRewardModel::getMaxReward(number agent_id, number) const
  {
    return this->max.at(agent_id);
  }

  double CompetitiveRewardModel::getMinReward(number agent_id, number) const
  {
    return this->min.at(agent_id);
  }

} // namespace sdm
