#pragma once

#include <sdm/types.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/joint.hpp>

namespace sdm
{

  template <typename TObservation>
  class BaseObservation : public Observation
  {
  public:
    BaseObservation();
    BaseObservation(const TObservation &item);
    virtual ~BaseObservation();

    virtual TObservation getObservation() const;
    virtual void setObservation(const TObservation &state);

    virtual std::string str() const;

  protected:
    TObservation observation_;
  };

  using DiscreteObservation = BaseObservation<number>;
  using DiscreteObservationString = BaseObservation<std::string>;
  using ContinuousObservation = BaseObservation<double>;

  const std::shared_ptr<Observation> DEFAULT_OBSERVATION = std::make_shared<DiscreteObservationString>("NoObs");

} // namespace sdm

#include <sdm/core/base_observation.tpp>
