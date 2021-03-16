/*=============================================================================
  Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <torch/torch.h>

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
namespace sdm
{


  const size_t SUCCESS = 0;
  const size_t ERROR_IN_COMMAND_LINE = 1;
  const size_t ERROR_UNHANDLED_EXCEPTION = 2;

  typedef std::size_t size_t; // = uint32_t

  typedef unsigned short number; // = uint16_t

  typedef unsigned short dtype;

  typedef unsigned short agent;

  typedef unsigned short state;

  typedef unsigned short action;

  typedef unsigned short horizon;

  typedef unsigned short observation;

  typedef boost::bimaps::bimap<std::string, sdm::size_t> bimap;

  typedef typename bimap::value_type name2index;

	typedef double reward;

	typedef torch::Tensor history;

	typedef torch::Tensor state_probability_distribution;

	typedef std::tuple<history, history, std::vector<history>, state_probability_distribution, sdm::action, sdm::action, reward, history, history, std::vector<history>, state_probability_distribution> transition;

	typedef std::tuple<torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, std::vector<torch::Tensor>, torch::Tensor> batch;
 
	
	
	
	typedef std::tuple<history, history, action, action, action, observation, observation, reward> pomdp_transition;

	typedef std::tuple<history, history, action, reward, history, history> pomdp_transition_simple;

	typedef std::tuple<torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor> pomdp_batch;

	typedef std::tuple<torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor> pomdp_batch_simple;

	typedef std::vector<pomdp_transition> pomdp_transitions_sequence;

  /**
  * Enumerator for the types of statistics that can be plotted.
  */
  enum Metric
  {
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
  enum Statistic
  {
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

  enum Criterion
  {
    COST_MIN,
    REW_MAX
  };

  class World;
  class State;
  class Action;
  class feedback;

  template <typename S, typename A>
  class UnaryFunction;

  template <typename S, typename A>
  class BinaryFunction;
} // namespace sdm
