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
  class BeliefState : public BaseState<TVector<std::shared_ptr<State>, double>>, public Distribution<std::shared_ptr<State>>
  {
  public:
    using state_type = std::shared_ptr<State>;
    using struct_type = TVector<std::shared_ptr<State>, double>;
    using value_type = typename struct_type::value_type;

    static double PRECISION;

    BeliefState();
    BeliefState(double);
    BeliefState(std::size_t, double);
    BeliefState(const BeliefState &);
    BeliefState(const TVector<std::shared_ptr<State>, double> &);
    BeliefState(std::initializer_list<value_type>);
    BeliefState(const std::vector<std::shared_ptr<State>> &list_states, const std::vector<double> &list_proba);

    void setProbabilityAt(const std::shared_ptr<State> &state, double proba);
    void addProbabilityAt(const std::shared_ptr<State> &state, double proba);
    double getProbabilityAt(const std::shared_ptr<State> &state) const;

    std::shared_ptr<State> sample() const;
    double getProbability(const std::shared_ptr<State> &begin, const std::shared_ptr<State> &end = 0) const;

    static std::shared_ptr<State> getState(const std::shared_ptr<State> &);

    bool operator==(const BeliefState &) const;

    std::string str() const;

    friend std::ostream &operator<<(std::ostream &os, BeliefState &belief)
    {
      os << "<belief " << belief.str() << "/>";
      return os;
    }

    template <class Archive>
    void serialize(Archive &archive, const unsigned int);
  };
} // namespace sdm

#include <sdm/core/state/belief_state.tpp>