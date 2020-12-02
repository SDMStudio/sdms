
/*=============================================================================
  Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <sdm/common.hpp>
#include <sdm/worlds.hpp>

//!
//! \file     algorithm.hpp
//! \author   Jilles S. Dibangoye
//! \brief    algorithm class
//! \version  1.0
//! \date     11 Avril 2016
//!
//! This class provides the algorithm' public interface.
//!

//! \namespace  sdm
//!
//! Namespace grouping all tools required for sequential decision making.
namespace sdm{
  class Algorithm{
  protected:
    double rate_end      = 0.0001;
    double rate_start    = 1.0;
    double rate_decay    = 100000;

    double epsilon;
    double eps_end      = 0.0001;
    double eps_start    = 1.0;
    double eps_decay    = 100000;

  public:
    virtual ~Algorithm();
    virtual void solve(const std::shared_ptr<POSG>&, horizon, double=0.001, double=1.0) = 0;
  };
}
