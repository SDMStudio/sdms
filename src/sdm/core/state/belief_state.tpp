#include <sdm/config.hpp>

namespace sdm
{
  double Belief::PRECISION = config::PRECISION_BELIEF;

  Belief::Belief()
  {
  }

  Belief::Belief(std::size_t size, double default_value)
  {
    this->container = std::make_shared<SparseVector<std::shared_ptr<State>>>(size, default_value);
  }

  Belief::Belief(std::initializer_list<Belief::value_type> list_values)
  {
    this->container = std::make_shared<SparseVector<std::shared_ptr<State>>>(list_values);
  }

  Belief::Belief(const std::vector<TState> &list_states, const std::vector<double> &list_proba) : Belief(list_states.size())
  {
    assert(list_states.size() == list_proba.size());
    for (size_t i = 0; i < list_states.size(); i++)
    {
      this->setProbabilityAt(list_states[i], list_proba[i]);
    }
  }

  Belief::Belief(const Belief &v) : TVector<TState, double>(v)
  {
  }

  Belief::Belief(const TVector<TState, double> &v) : TVector<TState, double>(v)
  {
  }

  void Belief::setProbabilityAt(const TState &state, double proba)
  {
    // Set the new occupancy measure
    this->setValueAt(state, proba);
  }

  double Belief::getProbabilityAt(const TState &state) const
  {
    return this->getValueAt(state);
  }

  void Belief::addProbabilityAt(const TState &state, double proba)
  {
    this->setValueAt(state, this->getProbabilityAt(state) + proba);
  }

  TState Belief::getState(const TState &state)
  {
    return state;
  }

  bool Belief::operator==(const std::shared_ptr<BeliefInterface> &other) const
  {
    if (this->size() != other->size())
    {
      return false;
    }
    for (const auto &state : this->getStates())
    {
      if (this->getProbability(state) != other->getProbability(state))
      {
        return false;
      }
    }
    return true;
  }

  bool Belief::operator==(const Belief &other) const
  {
    return TVector<TState, double>::is_equal(other, PRECISION);
  }

  std::string Belief::str() const
  {
    {
      std::ostringstream res;
      res << "[" << this->size() << "](";
      int i = 0;
      for (const auto &pair_state_proba : *this)
      {
        res << ((i == 0) ? "" : ", ");
        res << pair_state_proba.first << " --> " << pair_state_proba.second;
        i++;
      }
      res << ")";
      return res.str();
    }
  }

  template <class Archive>
  void Belief::serialize(Archive &archive, const unsigned int)
  {
    archive &boost::serialization::base_object<TVector<TState, double>>(*this);
  }

} // namespace sdm

namespace std
{

  struct hash<sdm::Belief<TState, TVector>>
  {
    typedef sdm::Belief<TState, TVector> argument_type;
    typedef std::size_t result_type;
    inline result_type operator()(const argument_type &in) const
    {
      return std::hash<TVector<TState, double>>()(in);
    }
  };
}