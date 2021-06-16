#include <sdm/core/dynamics/tabular_observation_dynamics_S.hpp>

namespace sdm
{

  TabularObservationDynamicsS::TabularObservationDynamicsS()
  {
  }

  TabularObservationDynamicsS::TabularObservationDynamicsS(const TabularObservationDynamicsS &copy)
      : observation_model_(copy.observation_model_),
        successor_observations_(copy.successor_observations_)
  {
  }
  
  TabularObservationDynamicsS::~TabularObservationDynamicsS(){}


  /* ############################################### */
  /* ### OBSERVATION_DYNAMICS -- P(O, S' | S, A) ### */
  /* ############################################### */

  double TabularObservationDynamicsS::getObservationProbability(const std::shared_ptr<State> &, const std::shared_ptr<Action> &, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number) const
  {
    return this->observation_model_.getValueAt(next_state, observation);
  }

  const MappedVector<std::shared_ptr<Observation>> &TabularObservationDynamicsS::getObservationProbabilities(const std::shared_ptr<State> &,const std::shared_ptr<Action> &, const std::shared_ptr<State> &next_state, number) const
  {
    return this->observation_model_.at(next_state);
  }

  void TabularObservationDynamicsS::setObservationProbability(const std::shared_ptr<State> &state,const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, double proba)
  {
    if (proba > 0)
    {
      this->setReachableObservations(state,action,next_state,observation);
      this->observation_model_.setValueAt(next_state, observation, proba);
    }
  }

  void TabularObservationDynamicsS::setObservationProbabilities(const std::shared_ptr<State> &,const std::shared_ptr<Action> &, const std::shared_ptr<State> &next_state, const MappedVector<std::shared_ptr<Observation>> &observation_probas)
  {
    this->observation_model_.setValuesAt(next_state, observation_probas);
  }

  void TabularObservationDynamicsS::setObservationModel(const MappedMatrix<std::shared_ptr<State>, std::shared_ptr<Observation>> &o_model)
  {
    this->observation_model_ = o_model;
  }

  std::set<std::shared_ptr<Observation>> TabularObservationDynamicsS::getReachableObservations(const std::shared_ptr<State> &, const std::shared_ptr<Action> &, const std::shared_ptr<State> &next_state, number) const
  {
    return this->successor_observations_.at(next_state);
  }


  void TabularObservationDynamicsS::setReachableObservations(const std::shared_ptr<State> &, const std::shared_ptr<Action> &, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number ) 
  {
    this->successor_observations_[next_state].insert(observation);
  }

  /* ################################### */
  /* ### DYNAMICS -- P(O, S' | S, A) ### */
  /* ################################### */

  // double TabularObservationDynamicsS::getDynamics(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number) const
  // {
  //   return this->getDynamics(state, action).getValueAt(next_state, observation);
  // }

  // void TabularObservationDynamicsS::setDynamics(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, double proba, number)
  // {
  //   this->dynamics_[state][action].setValueAt(next_state, observation, proba);
  // }

  // const MappedMatrix<std::shared_ptr<State>, std::shared_ptr<Observation>> &TabularObservationDynamicsS::getDynamics(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action) const
  // {
  //   return this->dynamics_.at(state).at(action);
  // }

} // namespace sdm
