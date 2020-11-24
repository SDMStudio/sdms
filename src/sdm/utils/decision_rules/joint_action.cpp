/*=============================================================================
Copyright (C) 2016 Jilles Steeve Dibangoye
==============================================================================*/

#include <sdm/types.hpp>
#include <sdm/utils/decision_rules/joint_action.hpp>

namespace sdm{
   joint_action::joint_action(const std::vector<agent>& set_agents, const std::vector<action>& decisions)
   : joint<action, 0>(set_agents.size(), decisions){}

   std::ostream& operator<<(std::ostream& os, const joint_action& ja){
     os << "<joint-action id=\"" << 1 << "\" />" << std::endl;
     agent ag = 0;
     for(ag=0; ag<ja.num_agents; ++ag)
      os << "<agent id=\"" << ag << "\" action=\"" << ja.cdecisions[ag] << "\"/>" << std::endl;

     os << "</joint-action>" << std::endl;
     return os;
   }
}
