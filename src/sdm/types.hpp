#pragma once

#include <cstddef>
#include <chrono>
#include <iostream>
#include <boost/bimap.hpp>

#include "sys/types.h"
#include "sys/sysinfo.h"

#include <sdm/config.hpp>

namespace sdm
{

  template <class I, class T>
  std::shared_ptr<T> cast(const std::shared_ptr<I> &item)
  {
    return std::static_pointer_cast<T>(item);
  }

  // namespace ptr
  // {
  //   template <typename T>
  //   using shared = std::shared_ptr<T>;

  //   template <typename T>
  //   using unique = std::unique_ptr<T>;

  //   template <typename T>
  //   using weak = std::weak_ptr<T>;
  // } // namespace ptr

  const size_t SUCCESS = 0;
  const size_t ERROR_IN_COMMAND_LINE = 1;
  const size_t ERROR_UNHANDLED_EXCEPTION = 2;

  typedef std::size_t size_t; // = uint32_t

  typedef unsigned short number; // = uint16_t

  typedef boost::bimaps::bimap<std::string, sdm::size_t> bimap;

  template <typename TItem>
  using bimap_item_index = boost::bimaps::bimap<TItem, sdm::size_t>;

  typedef typename bimap::value_type name2index;

  enum StateType
  {
    COMPRESSED,
    ONE_STEP_UNCOMPRESSED,
    FULLY_UNCOMPRESSED,
    COMPRESSED_KEEP_ALL,
    ONE_STEP_KEEP_ALL,
  };

  const std::unordered_map<std::string, StateType> STATE_TYPE_MAP = {
      {"COMPRESSED", StateType::COMPRESSED},
      {"ONE_STEP_UNCOMPRESSED", StateType::ONE_STEP_UNCOMPRESSED},
      {"FULLY_UNCOMPRESSED", StateType::FULLY_UNCOMPRESSED},
      {"COMPRESSED_KEEP_ALL", StateType::COMPRESSED_KEEP_ALL},
      {"ONE_STEP_KEEP_ALL", StateType::ONE_STEP_KEEP_ALL},
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
    RELAXED_SAWTOOTH_LINER_PROGRAMMING,
    RELAXED_V2_SAWTOOTH_LINER_PROGRAMMING

  };

  namespace MaxplanPruning
  {
    enum Type
    {
      PAIRWISE,
      BOUNDED,
      NONE
    };

    const std::unordered_map<std::string, MaxplanPruning::Type> TYPE_MAP = {
        {"PAIRWISE", MaxplanPruning::PAIRWISE},
        {"BOUNDED", MaxplanPruning::BOUNDED},
        {"NONE", MaxplanPruning::NONE},
    };
  }

  namespace SawtoothPruning
  {
    enum Type
    {
      PAIRWISE,
      NONE
    };

    const std::unordered_map<std::string, SawtoothPruning::Type> TYPE_MAP = {
        {"PAIRWISE", SawtoothPruning::PAIRWISE},
        {"NONE", SawtoothPruning::NONE},
    };
  }

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

  // using boost::hash_combine
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

  template <class BaseClass, class MyClass>
  std::shared_ptr<BaseClass> isInstanceOf(std::shared_ptr<MyClass> aPtr)
  {
    return std::dynamic_pointer_cast<BaseClass>(aPtr);
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
      return ((left == right) || left->isEqual(right));
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

  // struct Compare
  // {
  //   bool operator()(const std::string &a, const std::string &b) const
  //   {
  //     return a.length() < b.length();
  //   }
  // };

  struct Performance
  {

    static long long RanMemoryUsed(struct sysinfo memInfo)
    {
      sysinfo(&memInfo);

      long long physMemUsed = memInfo.totalram - memInfo.freeram;
      // Multiply in next statement to avoid int overflow on right hand side...
      physMemUsed *= memInfo.mem_unit;

      return physMemUsed / 1024 / 1024;
    }

    static long long totalMemory(struct sysinfo memInfo)
    {
      sysinfo(&memInfo);

      long long totalPhysMem = memInfo.totalram;
      // Multiply in next statement to avoid int overflow on right hand side...
      totalPhysMem *= memInfo.mem_unit;

      return totalPhysMem / 1024 / 1024;
    }

    // static void StartTime()
    // {
    //     Performance::time_start = std::chrono::high_resolution_clock::now();
    // }

    static double computeTime(std::chrono::high_resolution_clock::time_point start_time)
    {
      return std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - start_time).count();
    }
  };

} // namespace std