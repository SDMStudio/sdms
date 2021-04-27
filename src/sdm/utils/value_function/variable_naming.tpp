namespace sdm
{
  template <typename TVector, typename TAction, typename TValue>
  std::string VarNaming<TVector, TAction, TValue>::getVarNameJointHistoryDecisionRule(action a, typename TVector::jhistory_type jh)
  {
    std::ostringstream oss;
    oss << "jdr" << "." << a  << "." << jh;
    return oss.str();
  }

  template <typename TVector, typename TAction, typename TValue>
  void VarNaming<TVector, TAction, TValue>::setNumber(const std::string& name, number id)
  {
    if( this->variables.find(name) == this->variables.end())
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
    oss << "weight" << "." << index;
    return oss.str();
  }

  template <typename TVector, typename TAction, typename TValue>
  std::string VarNaming<TVector, TAction, TValue>::getVarNameIndividualHistoryDecisionRule(action a, typename TVector::jhistory_type::element_type::ihistory_type ih, agent ag)
  {
    std::ostringstream oss;
    oss << "idr" << "." << a  << "." << ih << "." << ag;
    return oss.str();
  }

  template <typename TVector, typename TAction, typename TValue>
  number VarNaming<TVector, TAction, TValue>::getNumber(const std::string& name)
  {
    if( this->variables.find(name) == this->variables.end() )
    {
      std::cerr << "something went wrong in here!"<< "\tname:" << name << std::endl;
      exit(-1);
    }
    return this->variables.at( name );
  }
}