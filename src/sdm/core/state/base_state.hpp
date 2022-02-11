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


  /**
   * @brief A base class inheriting from the State interface.
   * 
   * @tparam TState the type of data used for the state.
   * 
   * This class can be used to instantiate a state represented by any type.
   * 
   * ```cpp
   * BaseState<char> state('a'), state2('b'); // Instanciate a state stored as a character.   
   * BaseState<float> float_state(0.0), float_state2(0.1); // Instanciate a state stored as a float.   
   * ```
   * 
   */
  template <typename TData>
  class BaseState : public State
  {
  public:
    BaseState();
    BaseState(const TData &item);
    virtual ~BaseState();

    /**
     * @brief Get the data corresponding to the stored state.
     * 
     * @return the data
     */
    virtual TData getState() const;

    /**
     * @brief Set the data corresponding to the stored state.
     * 
     * @param state the data
     */
    virtual void setState(const TData &state);

    virtual std::string str() const;

    virtual bool operator==(const BaseState&other) const;
    
    TData state;
  };

    /** @brief BaseState class with type `number` */
    using DiscreteState = BaseState<number>;
    /** @brief BaseState class with type `std::string` */
    using StringState = BaseState<std::string>;
    /** @brief BaseState class with type `double` */
    using ContinuousState = BaseState<double>;

} // namespace sdm

#include <sdm/core/state/base_state.tpp>
