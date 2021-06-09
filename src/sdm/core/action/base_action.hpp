/**
 * @file discrete_action.hpp
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
#include <sdm/core/action/action.hpp>
#include <sdm/core/joint.hpp>

namespace sdm
{

  template <typename TAction>
  class BaseAction : public Action
  {
  public:
    BaseAction() {}
    BaseAction(const TAction &action) : action_(action) {}
    virtual ~BaseAction() {}

    virtual TAction getAction() const { return this->action_; }
    virtual void setAction(const TAction &action) { this->action_ = action; }

    virtual std::string str() const
    {
      std::ostringstream res;
      res << "Action(" << this->action_ << ")";
      return res.str();
    }

  protected:
    TAction action_;
  };

  using DiscreteAction = BaseAction<number>;
  using JointAction = BaseAction<Joint<std::shared_ptr<Action>>>;

} // namespace sdm
