#include <iomanip>
#include <sdm/config.hpp>
#include <sdm/exception.hpp>
#include <sdm/core/state/belief_state.hpp>
#include <sdm/utils/linear_algebra/hyperplane/alpha_vector.hpp>
#include <sdm/utils/linear_algebra/hyperplane/beta_vector.hpp>

namespace sdm
{
  double Belief::PRECISION = config::PRECISION_BELIEF;

  Belief::Belief() : container(0.)
  {
  }

  Belief::Belief(std::size_t size) : container(size, 0.)
  {
  }

  Belief::Belief(const std::vector<std::shared_ptr<State>> &list_states, const std::vector<double> &list_proba) : Belief(list_states.size())
  {
    assert(list_states.size() == list_proba.size());
    for (size_t i = 0; i < list_states.size(); i++)
    {
      this->setProbability(list_states[i], list_proba[i]);
    }
  }

  Belief::Belief(const Belief &v) : container(v.container)
  {
  }

  Belief::Belief(const MappedVector<std::shared_ptr<State>> &container) : container(container)
  {
    this->finalize();
  }

  Belief::~Belief()
  {
  }

  std::vector<std::shared_ptr<State>> Belief::getStates() const
  {
    return this->container.getIndexes();
  }

  void Belief::setProbability(const std::shared_ptr<State> &state, double proba)
  {
    // Set the new occupancy measure
    this->container.setValueAt(state, proba);
  }

  double Belief::getProbability(const std::shared_ptr<State> &state) const
  {
    return this->container.getValueAt(state);
  }

  double Belief::getProbability(const std::shared_ptr<State> &state, const std::shared_ptr<State> &) const
  {
    return this->getProbability(state);
  }

  void Belief::addProbability(const std::shared_ptr<State> &state, double proba)
  {
    this->container.setValueAt(state, this->getProbability(state) + proba);
  }

  std::shared_ptr<State> Belief::sampleState()
  {
    return this->sample();
  }

  std::shared_ptr<State> Belief::sample() const
  {
    // Get a random number between 0 and 1
    double epsilon = std::rand() / (double(RAND_MAX)), cumul;

    for (auto pair_item_proba : this->container)
    {
      cumul += pair_item_proba.second;
      if (epsilon < cumul)
      {
        return pair_item_proba.first;
        break;
      }
    }
    throw sdm::exception::Exception("Incomplete Distribution");
  }

  void Belief::normalizeBelief(double norm_1)
  {
    if (norm_1 > 0)
    {
      for (const auto &state : this->getStates())
      {
        this->setProbability(state, this->getProbability(state) / norm_1);
      }
    }
  }

  size_t Belief::hash(double precision) const
  {
    if (precision < 0)
    {
      precision = Belief::PRECISION;
    }
    return std::hash<MappedVector<std::shared_ptr<State>>>()(this->container, precision);
  }

  bool Belief::isEqualNorm1(const std::shared_ptr<BeliefInterface> &other, double precision) const
  {
    // return sdm::Norm1Equality(left, right, PWLCQValueFunction::GRANULARITY);
    double norm_1 = 0., additional = 1., proba_other;
    // For all points in the support
    for (const auto &state : this->getStates())
    {
      proba_other = other->getProbability(state);
      additional -= proba_other;
      norm_1 += std::abs(this->getProbability(state) - proba_other);
      if (norm_1 > precision)
        return false;
    }

    return (((norm_1 + additional) / 2) - 1e-5 <= precision);
  }

  bool Belief::isEqual(const Belief &other, double precision) const
  {
    if (precision < 0)
    {
      precision = Belief::PRECISION;
    }
    return this->container.isEqual(other.container, precision);
  }

  bool Belief::isEqual(const std::shared_ptr<State> &other, double precision) const
  {
    return this->isEqual(*std::dynamic_pointer_cast<Belief>(other), precision);
  }

  bool Belief::operator==(const Belief &other) const
  {
    for (const auto &item : this->container)
    {
      if (item.second != other.getProbability(item.first))
      {
        return false;
      }
    }
    return true;
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

  Belief Belief::add(const Belief &other, double coef_this, double coef_other) const
  {
    Belief res;
    res.container = this->container.add(other.container, coef_this, coef_other);
    res.finalize();
    return res;
  }

  double Belief::operator^(const Belief &other) const
  {
    double product = 0.0;

    for (const auto &item : this->container)
    {
      product += item.second * other.getProbability(item.first);
    }
    return product;
  }

  double Belief::operator^(const std::shared_ptr<BeliefInterface> &other) const
  {
    double product = 0.0;

    for (const auto &item : this->container)
    {
      product += item.second * other->getProbability(item.first);
    }
    return product;
  }

  double Belief::product(const std::shared_ptr<AlphaVector> &alpha) 
  {
    double product = 0.0;

    for (const auto &item : this->container)
    {
      product += item.second * alpha->getValueAt(item.first, nullptr);
    }
    return product;
  }

  double Belief::product(const std::shared_ptr<BetaVector> &beta, const std::shared_ptr<Action> &action) 
  {
    double product = 0.0;

    for (const auto &item : this->container)
    {
      product += item.second * beta->getValueAt(item.first, nullptr, action);
    }
    return product;
  }

  double Belief::norm_1() const
  {
    return this->container.norm_1();
  }

  TypeState Belief::getTypeState() const
  {
    return TypeState::BELIEF_STATE;
  }

  void Belief::setDefaultValue(double default_value)
  {
    this->container.setDefault(default_value);
  }

  double Belief::getDefaultValue() const
  {
    return this->container.getDefault();
  }

  void Belief::finalize()
  {
    this->container.finalize();
  }

  size_t Belief::size() const
  {
    return this->container.size();
  }

  std::string Belief::str() const
  {
    std::ostringstream res;
    res << std::setprecision(config::BELIEF_DECIMAL_PRINT) << std::fixed;

    res << "BeliefState[" << this->container.size() << "]( ";
    int i = 0;
    for (const auto &pair_state_proba : this->container)
    {
      res << ((i == 0) ? "" : " | ");
      res << pair_state_proba.first->str() << " : " << pair_state_proba.second;
      i++;
    }
    res << ", default value =";
    res << this->getDefaultValue() << " )";
    return res.str();
  }

} // namespace sdm
