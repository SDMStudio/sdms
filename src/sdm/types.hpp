/*=============================================================================
  Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <cstddef>
#include <iostream>
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

  template <class I, class T>
  std::shared_ptr<T> cast(const std::shared_ptr<I> &item)
  {
    return std::static_pointer_cast<T>(item);
  }

  namespace ptr
  {
    template <typename T>
    using shared = std::shared_ptr<T>;

    template <typename T>
    using unique = std::unique_ptr<T>;

    template <typename T>
    using weak = std::weak_ptr<T>;
  } // namespace ptr

  const size_t SUCCESS = 0;
  const size_t ERROR_IN_COMMAND_LINE = 1;
  const size_t ERROR_UNHANDLED_EXCEPTION = 2;

  typedef std::size_t size_t; // = uint32_t

  typedef unsigned short number; // = uint16_t

  typedef boost::bimaps::bimap<std::string, sdm::size_t> bimap;

  template <typename TItem>
  using bimap_item_index = boost::bimaps::bimap<TItem, sdm::size_t>;

  typedef typename bimap::value_type name2index;

  enum TState_t
  {
    COMPRESSED,
    ONE_STEP_UNCOMPRESSED,
    FULLY_UNCOMPRESSED
  };

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

  enum TypeOfResolution
  {
    BigM,
    IloIfThenResolution
  };

  enum TypeSawtoothLinearProgram
  {
    PLAIN_SAWTOOTH_LINER_PROGRAMMING,
    RELAXED_SAWTOOTH_LINER_PROGRAMMING
  };

  enum TypeOfMaxPlanPrunning
  {
    PAIRWISE,
    BOUNDED
  };

  // std::ostream& operator<<(std::ostream& out, const TypeOfMaxPlanPrunning value)
  // {
  //   const char* s = 0;

  //   switch (value)
  //   {
  //   case TypeOfMaxPlanPrunning::PAIRWISE :
  //     s = "PAIRWISE";
  //   case TypeOfMaxPlanPrunning::BOUNDED :    
  //     s = "BOUNDED";

  //   default:
  //     break;
  //   }
  //   return out<<s;
  // }

  enum TypeOfSawtoothPrunning
  {
    ITERATIVE,
    GLOBAL,
    BOTH,
    NONE
  };


  enum TypeState
  {
    STATE,
    BELIEF_STATE,
    OCCUPANCY_STATE,
    SERIAL_OCCUPANCY_STATE,
    SERIAL_STATE,
    UNKNOWN
  };

  enum TypeAction
  {
    ACTION,
    DECISION_RULE
  };

  enum
  {
    STANDARD,
    XML,
    JSON
  };

  //using boost::hash_combine
  template <class T>
  inline void hash_combine(std::size_t &seed, T const &v)
  {
    seed ^= std::hash<T>()(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  }

  template <>
  inline void hash_combine(std::size_t &seed, double const &v)
  {
    float tr_double = (float)v;
    sdm::hash_combine(seed, tr_double);
  }

  // template <typename T>
  // struct equal_from_ptr
  // {
  //   virtual bool operator()(const std::shared_ptr<void> &left, const std::shared_ptr<void> &right) const
  //   {
  //     auto casted_left = std::static_pointer_cast<T>(left);
  //     auto casted_right = std::static_pointer_cast<T>(right);
  //     return ((left == right) || std::equal_to<T>()(*casted_left, *casted_right));
  //   }
  // };

  // template <typename T>
  // struct hash_from_ptr
  // {
  //   virtual size_t operator()(const std::shared_ptr<void> &item) const
  //   {
  //     auto casted_item = std::static_pointer_cast<T>(item);
  //     return std::hash<T>()(*casted_item);
  //   }
  // };

  template <typename T>
  struct equal_from_ptr
  {
    virtual bool operator()(const std::shared_ptr<T> &left, const std::shared_ptr<T> &right) const
    {
      // if left or right is a nullptr, then return false
      if ((left == nullptr) ^ (right == nullptr))
        return false;
      return ((left == right) || left->operator==(right));
    }
  };

  template <typename T>
  struct hash_from_ptr
  {
    virtual size_t operator()(const std::shared_ptr<T> &item) const
    {
      return (item == nullptr) ? 0 : item->hash();
    }
  };

} // namespace sdm

namespace std
{

  // template <typename T>
  // struct hash<std::shared_ptr<T>>
  // {
  //   typedef std::shared_ptr<T> argument_type;
  //   typedef std::size_t result_type;
  //   inline result_type operator()(const argument_type &in) const
  //   {
  //     return std::hash<T>(*in);
  //   }
  // };

  template <class T, class... Ts>
  struct is_any : std::disjunction<std::is_same<T, Ts>...>
  {
  };

  /* Trick to allow multiple inheritance of objects
 * inheriting shared_from_this.
 * cf. https://stackoverflow.com/a/12793989/587407
 */

  /* First a common base class
 * of course, one should always virtually inherit from it.
 */
  class MultipleInheritableEnableSharedFromThis : public std::enable_shared_from_this<MultipleInheritableEnableSharedFromThis>
  {
  public:
    virtual ~MultipleInheritableEnableSharedFromThis()
    {
    }
  };

  template <class T>
  class inheritable_enable_shared_from_this : virtual public MultipleInheritableEnableSharedFromThis
  {
  public:
    std::shared_ptr<T> shared_from_this()
    {
      return std::dynamic_pointer_cast<T>(MultipleInheritableEnableSharedFromThis::shared_from_this());
    }
    /* Utility method to easily downcast.
     * Useful when a child doesn't inherit directly from enable_shared_from_this
     * but wants to use the feature.
     */
    template <class Down>
    std::shared_ptr<Down> downcasted_shared_from_this()
    {
      return std::dynamic_pointer_cast<Down>(MultipleInheritableEnableSharedFromThis::shared_from_this());
    }
  };

  struct Compare
  {
    bool operator()(const std::string &a, const std::string &b) const
    {
      return a.length() < b.length();
    }
  };

} // namespace std