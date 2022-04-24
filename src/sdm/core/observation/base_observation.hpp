#pragma once

#include <sdm/types.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/joint.hpp>

namespace sdm
{

  /**
   * @brief A base class inheriting from the Observation interface.
   * 
   * @tparam TObservation the type of data used for the observation.
   * 
   * This class can be used to instantiate an observation represented by any type.
   * ```cpp
   * BaseObservation<char> obs('a'), obs2('b'); // Instanciate an observation stored as a character.   
   * BaseObservation<float> float_obs(0.0), float_obs2(0.1); // Instanciate an observation stored as a float.   
   * ```
   * 
   */
  template <typename TObservation>
  class BaseObservation : public Observation
  {
  public:
    BaseObservation();
    
    BaseObservation(const TObservation &item);

    virtual ~BaseObservation();

    /**
     * @brief Get the data corresponding to the stored observation.
     * 
     * @return the data
     */
    virtual TObservation getObservation() const;

    /**
     * @brief Set the data corresponding to the stored observation.
     * 
     * @param state the data
     */
    virtual void setObservation(const TObservation &state);

    virtual std::string str() const;

    bool operator==(const BaseObservation &other);
    

  protected:
    TObservation observation_;
  };

  /** @brief BaseObservation class with type `number` */
  using DiscreteObservation = BaseObservation<number>;
  /** @brief BaseObservation class with type `std::string` */
  using StringObservation = BaseObservation<std::string>;
  /** @brief BaseObservation class with type `double` */
  using ContinuousObservation = BaseObservation<double>;

} // namespace sdm

#include <sdm/core/observation/base_observation.tpp>
