/*=============================================================================
Copyright (C) 2016 Jilles Steeve Dibangoye
==============================================================================*/

#include <sdm/types.hpp>
#include <sdm/utils/decision_rules/joint_observation.hpp>

namespace sdm{
  joint_observation::joint_observation(const std::vector<agent>& set_agents, const std::vector<observation>& decisions)
  : joint<observation, 1>(set_agents.size(), decisions){}

  std::ostream& operator<<(std::ostream& os, const joint_observation& jz){
    os << "<joint-observation id=\"" << 1 << "\" />" << std::endl;
    agent ag = 0; for(ag=0; ag<jz.num_agents; ++ag)
     os << "<agent id=\"" << ag << "\" observation=\"" << jz.cdecisions[ag] << "\"/>" << std::endl;

    os << "</joint-observation>" << std::endl;
    return os;
  }
}
