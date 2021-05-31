/**
 * @file discrete_state.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief 
 * @version 1.0
 * @date 31/05/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/core/state/state.hpp>

namespace sdm
{

  template <typename TState>
  class BaseState : public State
  {
  public:
    BaseState() {}
    BaseState(const TState &item) : state_(state) {}

    virtual TState getState() const { return this->state_; }
    virtual void setState(const TState &state) { this->state_ = state; }

    virtual std::string str() const
    {
      std::ostringstream res;
      res << "S(" << this->state_ << ")";
      return res.str();
    }

  protected:
    TState state_;
  };

  using DiscreteState = BaseState<number>;
  using ContinuousState = BaseState<double>;
  using JointDiscreteState = BaseState<Joint<DiscreteState>>;
  using JointContinuousState = BaseState<Joint<ContinuousState>>;

} // namespace sdm
