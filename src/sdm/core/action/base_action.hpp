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

  /**
   * @brief A base class inheriting from the Action interface.
   * 
   * @tparam TAction the type of data used for the action.
   * 
   * This class can be used to instantiate an action represented by any type.
   * 
   * ```cpp
   * BaseState<char> action('a'), action2('b'); // Instanciate an action stored as a character.   
   * BaseState<float> float_action(0.0), float_action2(0.1); // Instanciate an action stored as a float.  
   * ``` 
   * 
   */
  template <typename TAction>
  class BaseAction : public Action
  {
  public:
    BaseAction() {}
    BaseAction(const TAction &action) : action_(action) {}
    virtual ~BaseAction() {}

    /**
     * @brief Get the data corresponding to the stored state.
     * 
     * @return the data
     */
    virtual TAction getAction() const { return this->action_; }

    /**
     * @brief Set the data corresponding to the stored state.
     * 
     * @param state the data
     */
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

  /** @brief BaseAction class with type `number` */
  using DiscreteAction = BaseAction<number>;
  /** @brief BaseAction class with type `Joint<std::shared_ptr<Action>>`.  */
  using JointAction = BaseAction<Joint<std::shared_ptr<Action>>>;

} // namespace sdm
