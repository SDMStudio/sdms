#include <sdm/core/observation/default_observation.hpp>
#include <sdm/core/observation/base_observation.hpp>

namespace sdm
{
    const std::shared_ptr<Observation> NO_OBSERVATION = std::make_shared<StringObservation>("NoObs");
} // namespace sdm
