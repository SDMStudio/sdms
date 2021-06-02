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
#include <sdm/utils/struct/pair.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/linear_algebra/sdms_vector.hpp>

namespace sdm
{
  class Belief : public BeliefInterface
  {
  public:
    static double PRECISION;

    Belief();
    Belief(double);
    Belief(std::size_t, double);
    Belief(const Belief &);
    Belief(const TVector<std::shared_ptr<State>, double> &);
    Belief(std::initializer_list<value_type>);
    Belief(const std::vector<std::shared_ptr<State>> &list_states, const std::vector<double> &list_proba);

    virtual std::set<std::shared_ptr<State>> getAllStates() const;
    virtual double getProbability(const std::shared_ptr<State> &state) const;
    virtual void setProbability(const std::shared_ptr<State> &state, double proba);
    virtual void addProbability(const std::shared_ptr<State> &state, double proba);

    static std::shared_ptr<State> getState(const std::shared_ptr<State> &);

    bool operator==(const Belief &) const;

    virtual std::string str() const;

    friend std::ostream &operator<<(std::ostream &os, Belief &belief)
    {
      os << "<belief " << belief.str() << "/>";
      return os;
    }

    template <class Archive>
    void serialize(Archive &archive, const unsigned int);

  protected:
    std::shared_ptr<Vector<std::shared_ptr<State>>> container_;
  };
} // namespace sdm




#include <sdm/core/state/belief_state.tpp>