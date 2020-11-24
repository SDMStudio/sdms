/*=============================================================================
  Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <cstddef>
#include <iostream>
#include <unordered_map>
#include <boost/bimap.hpp>

//!
//! \file     types.hpp
//! \author   Jilles S. Dibangoye
//! \brief    defining several types
//! \version  1.0
//! \date     12 Avril 2016
//!
//! This class provides basic type alias for dpomdp.
//!

//! \namespace  sdm
//!
//! Namespace grouping all tools required for sequential decision making.
namespace sdm{

  typedef std::size_t number;

  typedef unsigned short dtype;

  typedef unsigned short agent;

  typedef unsigned short state;

  typedef unsigned short action;

  typedef unsigned short horizon;

  typedef unsigned short observation;

  typedef boost::bimaps::bimap<std::string, number> bimap;

  typedef typename bimap::value_type name2index;

  /**
  * Enumerator for the types of statistics that can be plotted.
  */
  enum Metric {
    CUMULATIVE_REWARD_PER_STEP,
    CUMULATIVE_REWARD_PER_EPISODE,
    AVERAGE_EPISODE_REWARD,
    AVERAGE_EPISODE_VALUE,
    MEDIAN_EPISODE_REWARD,
    CUMULATIVE_STEPS_PER_EPISODE,
    STEPS_PER_EPISODE
  };

  /**
  * Enumerator for the types of statistics that can be recorded.
  */
  enum Statistic {
    MIN,
    MAX,
    MEAN,
    RANGE,
    VARIANCE,
    STANDARD_DEVIATION
  };

  enum TypeSoftmax
  {
      BELIEF,
      BEHAVIOR,
      OCCUPANCY
  };


  class World;
  class State;
  class Action;
  class feedback;

  template<typename S, typename A>
  class UnaryFunction;

  template<typename S, typename A>
  class BinaryFunction;

  class dpomdp;
}
