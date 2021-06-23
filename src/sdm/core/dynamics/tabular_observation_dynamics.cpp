#include <sdm/core/dynamics/tabular_observation_dynamics.hpp>

namespace sdm
{

    // TabularObservationDynamics::TabularObservationDynamics()
    // {
    // }

    // TabularObservationDynamics::TabularObservationDynamics(const TabularObservationDynamics &copy)
    //     : observation_model_(copy.observation_model_),
    //       successor_observations_(copy.successor_observations_)
    // {
    // }

    // TabularObservationDynamics::~TabularObservationDynamics(){}

    std::shared_ptr<Distribution<std::shared_ptr<Observation>>> TabularObservationDynamics::getNextObservationDistribution(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t)
    {
        auto next_observation_distribution = std::make_shared<DiscreteDistribution<std::shared_ptr<Observation>>>();
        for (const auto &reachable_observation : this->getReachableObservations(state, action, next_state, t))
        {
            next_observation_distribution->setProbability(reachable_observation, this->getObservationProbability(state, action, next_state, reachable_observation, t));
        }
        return next_observation_distribution;
    }

} // namespace sdm
