#include <sdm/core/dynamics/tabular_observation_dynamics_SAS.hpp>

namespace sdm
{

  TabularObservationDynamicsSAS::TabularObservationDynamicsSAS()
  {
  }

  TabularObservationDynamicsSAS::TabularObservationDynamicsSAS(const TabularObservationDynamicsSAS &copy)
      : observation_model_(copy.observation_model_),
        successor_observations_(copy.successor_observations_)
  {
  }
  
  TabularObservationDynamicsSAS::~TabularObservationDynamicsSAS(){}


  /* ############################################### */
  /* ### OBSERVATION_DYNAMICS -- P(O, S' | S, A) ### */
  /* ############################################### */

  double TabularObservationDynamicsSAS::getObservationProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number) const
  {
    return this->observation_model_.at(state).at(action).getValueAt(next_state, observation);
  }

  const MappedVector<std::shared_ptr<Observation>> &TabularObservationDynamicsSAS::getObservationProbabilities(const std::shared_ptr<State> &state,const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number) const
  {
    return this->observation_model_.at(state).at(action).at(next_state);
  }

  void TabularObservationDynamicsSAS::setObservationProbability(const std::shared_ptr<State> &state,const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, double proba)
  {
    if (proba > 0)
    {
      this->setReachableObservations(state,action,next_state,observation);
      this->observation_model_[state][action].setValueAt(next_state, observation, proba);
    }
  }

  void TabularObservationDynamicsSAS::setObservationProbabilities(const std::shared_ptr<State> &state,const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const MappedVector<std::shared_ptr<Observation>> &observation_probas)
  {
    this->observation_model_[state][action].setValuesAt(next_state, observation_probas);
  }

  void TabularObservationDynamicsSAS::setObservationModel(const std::unordered_map<std::shared_ptr<State>,std::unordered_map<std::shared_ptr<Action>, MappedMatrix<std::shared_ptr<State>, std::shared_ptr<Observation>>>> &o_model)
  {
    this->observation_model_ = o_model;
  }

  std::set<std::shared_ptr<Observation>> TabularObservationDynamicsSAS::getReachableObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number) const
  {
    return this->successor_observations_.at(state).at(action).at(next_state);
  }


  void TabularObservationDynamicsSAS::setReachableObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) 
  {
    this->successor_observations_[state][action][next_state].insert(observation);
  }

  std::shared_ptr<Distribution<std::shared_ptr<Observation>>> TabularObservationDynamicsSAS::getNextObservationDistribution(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state)
    {
        std::set<std::shared_ptr<Observation>> reachable_observations = this->getReachableObservations(state, action, next_state);
        DiscreteDistribution<std::shared_ptr<Observation>> next_observation_distribution;
        for (std::shared_ptr<Observation> reachable_observation : reachable_observations)
        {
            next_observation_distribution.setProbability(reachable_observation, this->getObservationProbability(state, action, next_state, reachable_observation));
        }
        return std::make_shared<DiscreteDistribution<std::shared_ptr<Observation>>>(next_observation_distribution);
    }

  /* ################################### */
  /* ### DYNAMICS -- P(O, S' | S, A) ### */
  /* ################################### */

  // double TabularObservationDynamicsSAS::getDynamics(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number) const
  // {
  //   return this->getDynamics(state, action).getValueAt(next_state, observation);
  // }

  // void TabularObservationDynamicsSAS::setDynamics(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, double proba, number)
  // {
  //   this->dynamics_[state][action].setValueAt(next_state, observation, proba);
  // }

  // const MappedMatrix<std::shared_ptr<State>, std::shared_ptr<Observation>> &TabularObservationDynamicsSAS::getDynamics(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action) const
  // {
  //   return this->dynamics_.at(state).at(action);
  // }

} // namespace sdm
