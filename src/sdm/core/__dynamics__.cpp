/*=============================================================================
Copyright (C) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>

#include <sdm/core/__dynamics__.hpp>
#include <sdm/utils/linear_algebra/matrix.hpp>

namespace sdm
{
  __dynamics__::__dynamics__()
  {
  }

  __dynamics__::__dynamics__(action num_jactions, observation num_jobservations, state num_states)
  {
    this->initDynamics(num_jactions, num_jobservations, num_states);
  }

  void __dynamics__::initDynamics(action num_jactions, observation num_jobservations, state num_states)
  {
    action a;
    observation o;

    for (a = 0; a < num_jactions; ++a)
    {
      this->dynamics.push_back(std::vector<Matrix>());
      this->t_model.push_back(Matrix(num_states, num_states));
      this->o_model.push_back(Matrix(num_states, num_jobservations));
      for (o = 0; o < num_jobservations; ++o)
      {
        this->dynamics[a].push_back(Matrix(num_states, num_states));
      }
    }
  }

  void __dynamics__::setTransitionProbability(state x, action u, state y, double prob, bool cumul)
  {
    if (cumul)
      this->t_model[u](x, y) += prob;
    else
      this->t_model[u](x, y) = prob;
  }

  double __dynamics__::getTransitionProbability(state x, action u, state y) const
  {
    return this->t_model[u](x, y);
  }

  const std::unordered_set<state> &__dynamics__::getStateSuccessors(state x, action u)
  {
    return this->successor_states.at(x).at(u);
  }

  void __dynamics__::setTransitions(const std::vector<Matrix> &t_model)
  {
    this->t_model = t_model;
  }

  const Matrix &__dynamics__::getTransitions(action u)
  {
    return this->t_model[u];
  }

  double __dynamics__::getObservationProbability(action u, observation z, state x) const
  {
    return this->o_model[u](x, z);
  }

  void __dynamics__::setObservationProbability(action u, observation z, state x, double prob)
  {
    this->o_model[u](x, z) = prob;
  }

  const std::unordered_set<observation> &__dynamics__::getObservationSuccessors(action u, state x)
  {
    return this->successor_observations.at(x).at(u);
  }

  void __dynamics__::setObservations(const std::vector<Matrix> &o_model)
  {
    this->o_model = o_model;
  }

  const Matrix &__dynamics__::getObservations(action u)
  {
    return this->o_model[u];
  }

  double __dynamics__::getDynamics(state s, action a, observation o, state s_) const
  {
    return dynamics[a][o](s, s_);
  }

  const Matrix &__dynamics__::getDynamics(action a, observation o) const
  {
    return dynamics[a][o];
  }

  void __dynamics__::setDynamics(action a, observation o, const Matrix &m)
  {
    this->dynamics[a][o] = m;
  }

  void __dynamics__::setDynamics(state s, action a, observation o, state s_, double prob)
  {
    this->dynamics[a][o](s, s_) = prob;
  }
} // namespace sdm
