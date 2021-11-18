#pragma once

#include <tuple>

#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>

namespace sdm
{
  class ExperienceMemoryInterface
  {
    public:
      using sars_transition = std::tuple<std::shared_ptr<State>, std::shared_ptr<Action>, double, std::shared_ptr<State>, std::shared_ptr<Action>>;
      virtual ~ExperienceMemoryInterface(){}
      virtual void push(const std::shared_ptr<State>& state, const std::shared_ptr<Action>& action, const double reward, const std::shared_ptr<State>& next_state, const std::shared_ptr<Action>& next_action, number t) = 0;
      virtual std::vector<sars_transition> sample(number t, int n = 1) = 0;
      virtual int size() = 0;
  };
}