/*=============================================================================
  Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <sdm/types.hpp>

#include <tuple>

//!
//! \file     feedback.hpp
//! \author   Jilles S. Dibangoye
//! \brief    observation class
//! \version  1.0
//! \date     11 Avril 2016
//!
//! This class provides the observations' public interface.
//!

//! \namespace  sdm
//!
//! Namespace grouping all tools required for sequential decision making.
namespace sdm{

  class feedback{
  protected:
    state s;
    double r;
    observation z;
    static double rmin, rmax;

  public:
    feedback();
    feedback(double, double);
    feedback(state, observation, double);

    virtual ~feedback();

    virtual state getState() const;
    virtual void setState(const state&);
    virtual double getReward() const;
    virtual void setReward(const double&);
    virtual double getNormalizedReward() const;
    virtual observation getObservation() const;
    virtual void setObservation(const observation&);
  };

}
