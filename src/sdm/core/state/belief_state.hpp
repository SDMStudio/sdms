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
  template <typename TState, template <typename TI, typename TV> class TVector = DenseVector>
  class BaseBeliefState : public TVector<TState, double>
  {
  public:
    using state_type = TState;
    using struct_type = TVector<TState, double>;
    using value_type = typename struct_type::value_type;

    static double PRECISION;

    BaseBeliefState();
    BaseBeliefState(double);
    BaseBeliefState(std::size_t, double);
    BaseBeliefState(const BaseBeliefState &);
    BaseBeliefState(const TVector<TState, double> &);
    BaseBeliefState(std::initializer_list<value_type>);
    BaseBeliefState(const std::vector<TState> &list_states, const std::vector<double> &list_proba);

    void setProbabilityAt(const TState &, double);
    void addProbabilityAt(const TState &, double);
    double getProbabilityAt(const TState &) const;

    static TState getState(const TState &);

    bool operator==(const BaseBeliefState &) const;

    std::string str() const;

    friend std::ostream &operator<<(std::ostream &os, BaseBeliefState &state)
    {
      os << "<belief " << state.str() << "/>";
      return os;
    }

    template <class Archive>
    void serialize(Archive &archive, const unsigned int);
  };

  template <typename TState = number>
  using BeliefState = BaseBeliefState<TState>;

} // namespace sdm

#include <sdm/core/state/belief_state.tpp>