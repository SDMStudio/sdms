/*=============================================================================
  Copyright (c) 2020 David Albert
==============================================================================*/
#pragma once

#include <random>
#include <sdm/types.hpp>
#include <sdm/core/item.hpp>

//!
//! \file     action.hpp
//! \author   David Albert
//! \brief    Action class
//! \version  1.0
//! \date     10 décembre 2020
//!
//! This class provides the conditional probability functions' public interface.
//!

//! \namespace  sdm
//!
//! Namespace grouping all tools required for sequential decision making.
namespace sdm
{

  class DiscreteAction : public Item<number>
  {
  public:
    DiscreteAction()
    {
    }

    DiscreteAction(number item) : Item(item)
    {
    }
  };
} // namespace sdm
