/*=============================================================================
  Copyright (c) 2016-19 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <vector>

#include <sdm/types.hpp>
#include <sdm/core/__reward__.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>

//!
//! \file     constraint.hpp
//! \author   Jilles S. Dibangoye
//! \brief    constraint class
//! \version  1.0
//! \date     30 January 2019
//!
//! This class provides getter and setter methods for the constraint model.
//!

//! \namespace  sdm
//!
//! Namespace grouping all tools required for sequential decision making.
namespace sdm{

  //!
  //! \class  __constraint__  __constraint__.hpp
  //!
  class __constraint__ : public __reward__ {
    protected:
      double bound;

    public:
      //! \fn       value getCost(state, action) const
      //! \param    state
      //! \param    action
      //! \brief    Returns cost
      //! \return   value
      double getCost(state, action);

      double getBound() const;
  };

}
