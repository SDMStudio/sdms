#include <sdm/utils/value_function/backup/variable_naming.hpp>

namespace sdm
{
//   std::string VarNaming::getVarNameJointHistoryDecisionRule(int action, const typename TVector::jhistory_type &joint_history)
//   {
//     std::ostringstream oss;
//     oss << "jdr"
//         << "." << action << "." << joint_history->short_str();
//     return oss.str();
//   }

  std::string VarNaming::getVarNameJointHistoryDecisionRule(const std::shared_ptr<Action>& action, const std::shared_ptr<JointHistoryInterface> &joint_history)
  {
    std::ostringstream oss;
    oss << "jdr"
        << "." << action->str() << "." << joint_history->short_str();
    return oss.str();
  }

  void VarNaming::setNumber(const std::string &name, number id)
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

//   std::string VarNaming::getVarNameWeight(number index)
//   {
//     std::ostringstream oss;
//     oss << "weight"
//         << "." << index;
//     return oss.str();
//   }

//   std::string VarNaming::getVarNameWeight(const TVector &state)
//   {
//     std::ostringstream oss;
//     oss << "weight"
//         << "." << std::make_shared<TVector>(state);
//     return oss.str();
//   }

//   std::string VarNaming::getVarNameIndividualHistoryDecisionRule(int action, const typename TVector::jhistory_type::element_type::ihistory_type &individual_history, const number &agent_id)
//   {
//     std::ostringstream oss;
//     oss << "idr"
//         << "." << action << "." << individual_history->short_str() << "." << agent_id;
//     return oss.str();
//   }

  std::string VarNaming::getVarNameIndividualHistoryDecisionRule(const std::shared_ptr<Action>& action, const std::shared_ptr<HistoryInterface>& individual_history, const number &agent_id)
  {
    std::ostringstream oss;
    oss << "idr"
        << "." << action->str() << "." << individual_history->short_str() << "." << agent_id;
    return oss.str();
  }

  number VarNaming::getNumber(const std::string &name)
  {
    if (this->variables.find(name) == this->variables.end())
    {
      std::cerr << "something went wrong in here!"
                << "\tname:" << name << std::endl;
      exit(-1);
    }
    return this->variables.at(name);
  }

//   /**
//    * @warning : deprecated !!!
//    */
//   std::string VarNaming::getVarNameWeightedStateJointHistory(const TVector &i, const typename TVector::state_type &s, const typename TVector::jhistory_type &jh)
//   {
//     std::ostringstream oss;
//     oss << "wsh"
//         << "." << &i << "." << s << "." << jh->short_str();
//     return oss.str();
//   }

//   std::string VarNaming::getVarNameWeightedStateJointHistory(const std::shared_ptr<TVector> &i, const typename TVector::state_type &s, const typename TVector::jhistory_type &jh)
//   {
//     std::ostringstream oss;
//     oss << "wsh"
//         << "." << i << "." << s << "." << jh->short_str();
//     return oss.str();
//   }
}