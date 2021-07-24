#pragma once

#include <tuple>

#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>

namespace sdm
{
  class ExperienceMemoryInterface
  {
    public:
      using sars_transition = std::tuple<number, std::shared_ptr<Observation>, std::shared_ptr<Action>, double, std::shared_ptr<Observation>, std::shared_ptr<Action>>;
      virtual void push(const std::shared_ptr<Observation>& observation, const std::shared_ptr<Action>& action, const double reward, const std::shared_ptr<Observation>& next_observation, const std::shared_ptr<Action>& next_action, number t) = 0;
      virtual std::vector<sars_transition> sample(number t, int n) = 0;
      virtual std::vector<sars_transition> sample(int n) = 0;
      virtual int size() = 0;
  };
}