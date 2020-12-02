/*=============================================================================
  Copyright (c) 2020 David Albert
==============================================================================*/
#pragma once

#include <vector>
#include <boost/bimap.hpp>
#include <sdm/types.hpp>

//!
//! \file     space.hpp
//! \author   David Albert
//! \brief    abstract space class
//! \version  1.0
//! \date     24 novembre 2020
//!
//! This is an abstract interface for Spaces.

//! \namespace  sdm
//! \brief Namespace grouping all tools required for sequential decision making.
namespace sdm
{
  //! \class  Space  space.hpp
  class Space
  {
  public:
    virtual std::vector<number> getDim() const = 0;
    virtual std::string str() const = 0;
  };
} // namespace sdm
