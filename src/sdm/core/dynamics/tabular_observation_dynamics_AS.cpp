#include <sdm/core/dynamics/tabular_observation_dynamics_AS.hpp>

namespace sdm
{

  TabularObservationDynamicsAS::TabularObservationDynamicsAS()
  {
  }

  TabularObservationDynamicsAS::TabularObservationDynamicsAS(const TabularObservationDynamicsAS &copy)
      : observation_model_(copy.observation_model_),
        successor_observations_(copy.successor_observations_),
        next_observations_distrib_(copy.next_observations_distrib_)
  {
  }

  TabularObservationDynamicsAS::~TabularObservationDynamicsAS() {}

  /* ############################################### */
  /* ### OBSERVATION_DYNAMICS -- P(O, S' | S, A) ### */
  /* ############################################### */

  double TabularObservationDynamicsAS::getObservationProbability(const std::shared_ptr<State> &, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number) const
  {
    return this->observation_model_.at(action).getValueAt(next_state, observation);
  }

  const MappedVector<std::shared_ptr<Observation>> &TabularObservationDynamicsAS::getObservationProbabilities(const std::shared_ptr<State> &, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number) const
  {
    return this->observation_model_.at(action).at(next_state);
  }

  void TabularObservationDynamicsAS::setObservationProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, double proba)
  {
    if (proba > 0)
    {
      this->setReachableObservations(state, action, next_state, observation);
      this->observation_model_[action].setValueAt(next_state, observation, proba);
      this->updateNextObsDistribution(state, action, next_state, observation, proba);
    }
  }

  void TabularObservationDynamicsAS::setObservationProbabilities(const std::shared_ptr<State> &, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const MappedVector<std::shared_ptr<Observation>> &observation_probas)
  {
    this->observation_model_[action].setValuesAt(next_state, observation_probas);
  }

  void TabularObservationDynamicsAS::setObservationModel(const std::unordered_map<std::shared_ptr<Action>, MappedMatrix<std::shared_ptr<State>, std::shared_ptr<Observation>>> &o_model)
  {
    this->observation_model_ = o_model;
  }

  std::set<std::shared_ptr<Observation>> TabularObservationDynamicsAS::getReachableObservations(const std::shared_ptr<State> &, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number) const
  {
    return this->successor_observations_.at(action).at(next_state);
  }

  void TabularObservationDynamicsAS::setReachableObservations(const std::shared_ptr<State> &, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number)
  {
    this->successor_observations_[action][next_state].insert(observation);
  }

  void TabularObservationDynamicsAS::updateNextObsDistribution(const std::shared_ptr<State> &, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, double proba)
  {
    // If the distribution does not already exists
    if (this->next_observations_distrib_.find(action) == this->next_observations_distrib_.end())
    {
      this->next_observations_distrib_.emplace(action, std::unordered_map<std::shared_ptr<State>, std::shared_ptr<DiscreteDistribution<std::shared_ptr<Observation>>>>());
    }
    // If the distribution does not already exists
    if (this->next_observations_distrib_[action].find(next_state) == this->next_observations_distrib_[action].end())
    {
      this->next_observations_distrib_[action].emplace(next_state, std::make_shared<DiscreteDistribution<std::shared_ptr<Observation>>>());
    }
    this->next_observations_distrib_[action][next_state]->setProbability(observation, proba);
  }

  std::shared_ptr<Distribution<std::shared_ptr<Observation>>> TabularObservationDynamicsAS::getNextObservationDistribution(const std::shared_ptr<State> &, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number)
  {
    return this->next_observations_distrib_.at(action).at(next_state);
  }

  /* ################################### */
  /* ### DYNAMICS -- P(O, S' | S, A) ### */
  /* ################################### */

  // double TabularObservationDynamicsAS::getDynamics(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number) const
  // {
  //   return this->getDynamics(state, action).getValueAt(next_state, observation);
  // }

  // void TabularObservationDynamicsAS::setDynamics(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, double proba, number)
  // {
  //   this->dynamics_[state][action].setValueAt(next_state, observation, proba);
  // }

  // const MappedMatrix<std::shared_ptr<State>, std::shared_ptr<Observation>> &TabularObservationDynamicsAS::getDynamics(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action) const
  // {
  //   return this->dynamics_.at(state).at(action);
  // }

} // namespace sdm
