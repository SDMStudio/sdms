/**
 * @file belief_state.cpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief
 * @version 1.0
 * @date 29/03/2021
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <algorithm>

#include <sdm/types.hpp>
#include <sdm/macros.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/state/interface/belief_interface.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/linear_algebra/sdms_vector.hpp>
#include <sdm/core/distribution.hpp>

namespace sdm
{
  class Belief : virtual public BeliefInterface,
                 public Distribution<std::shared_ptr<State>>
  {
  public:
    static double PRECISION;

    Belief();
    Belief(std::size_t);
    Belief(const Belief &);
    Belief(const MappedVector<std::shared_ptr<State>> &);
    Belief(const std::vector<std::shared_ptr<State>> &list_states, const std::vector<double> &list_proba);

    virtual ~Belief();

    std::vector<std::shared_ptr<State>> getStates() const;

    double getProbability(const std::shared_ptr<State> &state) const;
    double getProbability(const std::shared_ptr<State> &begin, const std::shared_ptr<State> &) const;

    void setProbability(const std::shared_ptr<State> &state, double proba);
    void addProbability(const std::shared_ptr<State> &state, double proba);

    Pair<std::shared_ptr<State>, double> next(const std::shared_ptr<MDPInterface> &mdp, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t);
    double getReward(const std::shared_ptr<MDPInterface> &mdp, const std::shared_ptr<Action> &action, number t);

    std::shared_ptr<State> sample() const;
    std::shared_ptr<State> sampleState();

    void normalizeBelief(double norm_1);

    size_t hash(double precision = PRECISION) const;

    bool isEqual(const Belief &other, double precision = PRECISION) const;
    bool isEqual(const std::shared_ptr<State> &other, double precision = PRECISION) const;

    bool operator==(const Belief &other) const;

    Belief add(const Belief &other, double coef_this = 1., double coef_other = 1.) const;

    double product(const std::shared_ptr<AlphaVector> &alpha);
    double product(const std::shared_ptr<BetaVector> &beta, const std::shared_ptr<Action> &action);

    double norm_1() const;
    bool isEqualNorm1(const std::shared_ptr<BeliefInterface> &other, double precision) const;

    TypeState getTypeState() const;

    void setDefaultValue(double);
    double getDefaultValue() const;

    void finalize();
    size_t size() const;

    std::string str() const;

    friend std::ostream &operator<<(std::ostream &os, const Belief &belief)
    {
      os << belief.str();
      return os;
    }

    template <class Archive>
    void serialize(Archive &archive, const unsigned int)
    {
      archive &boost::serialization::base_object<MappedVector<std::shared_ptr<State>, double>>(*this);
    }

  protected:
    MappedVector<std::shared_ptr<State>> container;
  };
} // namespace sdm

DEFINE_STD_HASH(sdm::Belief, sdm::Belief::PRECISION);