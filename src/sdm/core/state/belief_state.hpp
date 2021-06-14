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
#include <sdm/core/state/state.hpp>
#include <sdm/core/state/belief_interface.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/linear_algebra/sdms_vector.hpp>

namespace sdm
{
  class Belief : public BeliefInterface,
                 public MappedVector<std::shared_ptr<State>>
  {
  public:
    static double PRECISION;

    Belief();
    Belief(double);
    Belief(std::size_t, double);
    Belief(const Belief &);
    Belief(const MappedVector<std::shared_ptr<State>> &);
    // Belief(std::initializer_list<value_type>);
    Belief(const std::vector<std::shared_ptr<State>> &list_states, const std::vector<double> &list_proba);
    virtual ~Belief();

    std::vector<std::shared_ptr<State>> getStates() const;
    double getProbability(const std::shared_ptr<State> &state) const;
    void setProbability(const std::shared_ptr<State> &state, double proba);
    void addProbability(const std::shared_ptr<State> &state, double proba);

    static std::shared_ptr<State> getState(const std::shared_ptr<State> &);
    size_t size() const { return MappedVector<std::shared_ptr<State>>::size(); }

    bool operator==(const Belief &) const;

    std::string str() const;
    bool operator==(const std::shared_ptr<BeliefInterface> &other) const;

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
  };
} // namespace sdm

namespace std
{

  template <>
  struct hash<sdm::Belief>
  {
    typedef sdm::Belief argument_type;
    typedef std::size_t result_type;
    inline result_type operator()(const argument_type &in) const
    {
      return std::hash<sdm::MappedVector<std::shared_ptr<sdm::State>>>()(in);
    }
  };
}