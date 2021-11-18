#include <sdm/utils/linear_programming/variable_naming.hpp>

namespace sdm
{
    std::string VarNaming::getVarNameJointHistoryDecisionRule(const std::shared_ptr<Action>& action, const std::shared_ptr<JointHistoryInterface> &joint_history)
    {
      std::ostringstream oss;
      oss << "jdr"
          << "." << action << "." << joint_history;
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

    std::string VarNaming::getVarNameWeight(number index)
    {
      std::ostringstream oss;
      oss << "weight"
          << "." << index;
      return oss.str();
    }

    std::string VarNaming::getVarNameIndividualHistoryDecisionRule(const std::shared_ptr<Action>& action, const std::shared_ptr<HistoryInterface>& individual_history, const number &agent_id)
    {
      std::ostringstream oss;
      oss << "idr"
          << "." << action << "." << individual_history << "." << agent_id;
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

    std::string VarNaming::getVarNameWeightedStateJointHistory(const std::shared_ptr<State>&i, const std::shared_ptr<State>&s, const std::shared_ptr<JointHistoryInterface> &jh)
    {
      std::ostringstream oss;
      oss << "wsh"
          << "." << i << "." << s << "." << jh;
      return oss.str();
    }
    
    std::string VarNaming::getVarNameIndividualHistory(const std::shared_ptr<HistoryInterface>& ihistory, const number agent)
    {
      std::ostringstream oss;
      oss << "ih"
          << "." << ihistory << "." << agent;
      return oss.str();
    }

}