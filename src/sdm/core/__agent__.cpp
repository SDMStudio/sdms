#include <sdm/core/__agent__.hpp>

namespace sdm{

  __agent__::__agent__() {}
  
  __agent__::__agent__(agent num_agents) : num_agents_(num_agents){}
  
  __agent__::__agent__(const std::vector<std::string> & agent_names) : num_agents_(agent_names.size()){
    for (agent idx = 0; idx < this->num_agents_; ++idx)
      this->agent_names_bimap.insert(name2index(agent_names[idx], idx));
  }


  agent __agent__::getNumAgents() const{
    return this->num_agents_;
  }

  void __agent__::setNumAgents(agent number_agents){
    this->num_agents_ = number_agents;
  }

  void __agent__::setAgentsNames(const std::vector<std::string>& agent_names){
    if (this->num_agents_ == 0) {
      this->setNumAgents( agent_names.size() );
    }
    else {
      assert(this->num_agents_ == agent_names.size());
    }

    number idx =0;
    for(idx=0; idx<this->num_agents_; ++idx)
      this->agent_names_bimap.insert( name2index(agent_names[idx], idx) );
  }

  agent __agent__::getAgentIndex(const std::string& name){
    return this->agent_names_bimap.empty() ? 0 : this->agent_names_bimap.left.at(name);
  }

  std::string __agent__::getAgentName(agent ag){
    return this->agent_names_bimap.empty() ? std::to_string(ag) : this->agent_names_bimap.right.at(ag);
  }

}
