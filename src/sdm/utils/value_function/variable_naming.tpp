#include <memory>
namespace sdm
{
  template <typename TVector, typename TAction, typename TValue>
  std::string VarNaming<TVector, TAction, TValue>::getVarNameJointHistoryDecisionRule(int action, const typename TVector::jhistory_type &joint_history)
  {
    std::ostringstream oss;
    oss << "jdr"
        << "." << action << "." << joint_history->short_str();
    return oss.str();
  }

  template <typename TVector, typename TAction, typename TValue>
  std::string VarNaming<TVector, TAction, TValue>::getVarNameJointHistoryDecisionRule(typename TAction::output_type action, const typename TVector::jhistory_type &joint_history)
  {
    std::ostringstream oss;
    oss << "jdr"
        << "." << action << "." << joint_history->short_str();
    return oss.str();
  }

  template <typename TVector, typename TAction, typename TValue>
  void VarNaming<TVector, TAction, TValue>::setNumber(const std::string &name, number id)
  {
    if (this->variables.find(name) == this->variables.end())
    {
      this->variables.emplace(name, id);
    }
    else
    {
      this->variables.at(name) = id;
    }
  }

  template <typename TVector, typename TAction, typename TValue>
  std::string VarNaming<TVector, TAction, TValue>::getVarNameWeight(number index)
  {
    std::ostringstream oss;
    oss << "weight"
        << "." << index;
    return oss.str();
  }

  template <typename TVector, typename TAction, typename TValue>
  std::string VarNaming<TVector, TAction, TValue>::getVarNameWeight(const TVector &state)
  {
    std::ostringstream oss;
    oss << "weight"
        << "." << std::make_shared<TVector>(state);
    return oss.str();
  }

  template <typename TVector, typename TAction, typename TValue>
  std::string VarNaming<TVector, TAction, TValue>::getVarNameIndividualHistoryDecisionRule(int action, const typename TVector::jhistory_type::element_type::ihistory_type &individual_history, const number &agent_id)
  {
    std::ostringstream oss;
    oss << "idr"
        << "." << action << "." << individual_history->short_str() << "." << agent_id;
    return oss.str();
  }

  template <typename TVector, typename TAction, typename TValue>
  std::string VarNaming<TVector, TAction, TValue>::getVarNameIndividualHistoryDecisionRule(typename TAction::output_type action, const typename TVector::jhistory_type::element_type::ihistory_type &individual_history, const number &agent_id)
  {
    std::ostringstream oss;
    oss << "idr"
        << "." << action << "." << individual_history->short_str() << "." << agent_id;
    return oss.str();
  }

  template <typename TVector, typename TAction, typename TValue>
  number VarNaming<TVector, TAction, TValue>::getNumber(const std::string &name)
  {
    if (this->variables.find(name) == this->variables.end())
    {
      std::cerr << "something went wrong in here!"
                << "\tname:" << name << std::endl;
      exit(-1);
    }
    return this->variables.at(name);
  }

  /**
   * @warning : deprecated !!!
   */
  template <typename TVector, typename TAction, typename TValue>
  std::string VarNaming<TVector, TAction, TValue>::getVarNameWeightedStateJointHistory(const TVector &i, const typename TVector::state_type &s, const typename TVector::jhistory_type &jh)
  {
    std::ostringstream oss;
    oss << "wsh"
        << "." << &i << "." << s << "." << jh->short_str();
    return oss.str();
  }

  template <typename TVector, typename TAction, typename TValue>
  std::string VarNaming<TVector, TAction, TValue>::getVarNameWeightedStateJointHistory(const std::shared_ptr<TVector> &i, const typename TVector::state_type &s, const typename TVector::jhistory_type &jh)
  {
    std::ostringstream oss;
    oss << "wsh"
        << "." << i << "." << s << "." << jh->short_str();
    return oss.str();
  }
}