#pragma once

#include <unordered_map>
#include <ilcplex/ilocplex.h>
ILOSTLBEGIN

#include <sdm/core/state/occupancy_state.hpp>

//!
//! \file     lp_decentralized.hpp
//! \author   Jilles S. Dibangoye
//! \brief    lower_bound class
//! \version  1.0
//! \date     12 August 2018
//!
//! This class provides getter and setter methods to characterize decentralized policies using MILP.
//!

//! \namespace  sdm
//!
//! Namespace grouping all tools required for sequential decision making.
namespace sdm{
  template<typename ihistory, typename jhistory>
  class lp_decentralized {
  public:
    std::shared_ptr<Action> getDecentralizedVariables(const IloCplex&, const IloNumVarArray&, const std::shared_ptr<occupancy_map<jhistory>>&);

    void setDecentralizedVariables(const std::shared_ptr<occupancy_map<jhistory>>&, std::unordered_map<agent, std::unordered_set<ihistory*>>&, IloEnv&, IloNumVarArray&, number&) const;

    void setDecentralizedConstraints(const std::shared_ptr<occupancy_map<jhistory>>&, std::unordered_map<agent, std::unordered_set<ihistory*>>&, IloEnv&, IloRangeArray&, IloNumVarArray&, number&) const;
  };
}
