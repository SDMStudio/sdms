/**=============================================================================
  Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <sdm/types.hpp>

//!
//! \file     state.hpp
//! \author   Jilles S. Dibangoye
//! \brief    state class
//! \version  1.0
//! \date     11 Avril 2016
//!
//! This class provides the states' public interface.
//!

//! \namespace  sdm
//!
//! Namespace grouping all tools required for sequential decision making.
namespace sdm
{

  class World
  {
  protected:
    // std::shared_ptr<State> internal = 0;

  public:
    // virtual ~World();
    // virtual state init() = 0;
    // virtual number getNumAgents() const = 0;
    // virtual void execute(action, feedback *) = 0;
  };
} // namespace sdm
