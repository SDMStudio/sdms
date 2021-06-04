#include <sdm/core/dynamics/tabular_observation_dynamics.hpp>

namespace sdm
{

  TabularObservationDynamics::TabularObservationDynamics()
  {
  }

  TabularObservationDynamics::TabularObservationDynamics(const TabularObservationDynamics &copy) : observation_model_(copy.observation_model_), dynamics_(copy.dynamics_), successor_observations_(copy.successor_observations_)
  {
  }

  TabularObservationDynamics::TabularObservationDynamics(number num_jactions, number num_jobservations, number num_states)
  {
    this->initDynamics(num_jactions, num_jobservations, num_states);
  }

  void TabularObservationDynamics::initDynamics(number num_jactions, number num_jobservations, number num_states)
  {
    // number a;
    // number o;

    // for ; a < num_jactions; ++a)
    // {
    //   this->dynamics_.push_back(std::vector<DenseMatrix<number, number>>());
    //   this->observation_model_.push_back(DenseMatrix<number, number>(num_states, num_jobservations));
    //   for ; o < num_jobservations; ++o)
    //   {
    //     this->dynamics_[a].push_back(DenseMatrix<number, number>(num_states, num_states));
    //   }
    // }
  }


  /* ############################################### */
  /* ### OBSERVATION_DYNAMICS -- P(O, S' | S, A) ### */
  /* ############################################### */

  double TabularObservationDynamics::getObservationProbability(const std::shared_ptr<State> &, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number) const
  {
    return this->observation_model_.at(action).getValueAt(next_state, observation);
  }

  const MappedVector<std::shared_ptr<Observation>> &TabularObservationDynamics::getObservationProbabilities(const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number) const
  {
    return this->observation_model_.at(action).at(next_state);
  }

  void TabularObservationDynamics::setObservationProbability(const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, double proba)
  {
    if (proba > 0)
    {
      this->successor_observations_[action][next_state].insert(observation);
      this->observation_model_[action].setValueAt(next_state, observation, proba);
    }
  }

  void TabularObservationDynamics::setObservationProbabilities(const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const MappedVector<std::shared_ptr<Observation>> &observation_probas)
  {
    this->observation_model_[action].setValuesAt(next_state, observation_probas);
  }

  void TabularObservationDynamics::setObservationModel(const std::unordered_map<std::shared_ptr<Action>, MappedMatrix<std::shared_ptr<State>, std::shared_ptr<Observation>>> &o_model)
  {
    this->observation_model_ = o_model;
  }

  std::set<std::shared_ptr<Observation>> TabularObservationDynamics::getReachableObservations(const std::shared_ptr<State> &, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number) const
  {
    return this->successor_observations_.at(action).at(next_state);
  }


  /* ################################### */
  /* ### DYNAMICS -- P(O, S' | S, A) ### */
  /* ################################### */

  double TabularObservationDynamics::getDynamics(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number) const
  {
    return this->getDynamics(state, action).getValueAt(next_state, observation);
  }

  void TabularObservationDynamics::setDynamics(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, double proba, number)
  {
     this->dynamics_[state][action].setValueAt(next_state, observation, proba);
  }

  const MappedMatrix<std::shared_ptr<State>, std::shared_ptr<Observation>> &TabularObservationDynamics::getDynamics(const std::shared_ptr<State>& state, const std::shared_ptr<Action>& action) const
  {
    return this->dynamics_.at(state).at(action);
  }


} // namespace sdm
