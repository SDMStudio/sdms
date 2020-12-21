/**=============================================================================
  Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <string>

#include <sdm/types.hpp>
#include <sdm/world/posg.hpp>
#include <sdm/utils/value_function/pwlc_value_function.hpp>

//!
//! \file     generic_hsvi.hpp
//! \author   Jilles S. Dibangoye
//! \brief    state class
//! \version  1.0
//! \date     11 Avril 2016
//!
//! This class provides the hsvi' public interface.
//!

//! \namespace  sdm
//!
//! Namespace grouping all tools required for sequential decision making.
namespace sdm
{

  class GenericHSVI
  {
  protected:
    // ValueRepr value_function;
    PWLCValueFunction value_;

    std::shared_ptr<POSG> world;

    int trial, MAX_TRIALS;
    double epsilon;

    void initLogger();

    void initBounds();

    void updateBounds(TState s);

  public:
    GenericHSVI(number trials, std::string results);

    void solve(const std::shared_ptr<POSG> &world, number planning_horizon, double epsilon, double discount);

    bool stop(number h);

    void explore(Vector s, double r, number h);
  };
} // namespace sdm
