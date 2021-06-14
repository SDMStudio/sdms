/**
 * @file base_state.hpp
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
#include <sdm/core/joint.hpp>

namespace sdm
{

  template <typename TState>
  class BaseState : public State
  // public BaseItem<TState>
  {
  public:
    BaseState();
    BaseState(const TState &item);
    virtual ~BaseState();

    virtual TState getState() const;
    virtual void setState(const TState &state);

    virtual std::string str() const;

    virtual bool operator==(const BaseState&other) const;

  protected:
    TState state_;
  };

  using DiscreteState = BaseState<number>;
  using DiscreteStateString = BaseState<std::string>;
  using ContinuousState = BaseState<double>;

} // namespace sdm

#include <sdm/core/state/base_state.tpp>
