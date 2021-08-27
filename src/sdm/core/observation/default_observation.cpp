#include <sdm/core/observation/default_observation.hpp>
#include <sdm/core/base_observation.hpp>

namespace sdm
{
    const std::shared_ptr<Observation> NO_OBSERVATION = std::make_shared<DiscreteObservationString>("NoObs");
} // namespace sdm
