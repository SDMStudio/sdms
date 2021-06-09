#pragma once

#include <vector>

#include <sdm/types.hpp>
#include <sdm/tools.hpp>
#include <sdm/core/function.hpp>
#include <sdm/core/item.hpp>
#include <sdm/utils/struct/vector.hpp>

namespace sdm
{
  /**
   * @brief This class is used for joint objects. It can be a JointHistoryTree, a JointObservation, a JointAction, etc.
   * 
   * @tparam The type of item.
   */
  template <typename T>
  class BaseJoint : public Item, public std::vector<T>, public Function<number, T>
  {
  public:
    using value_type = T;

    BaseJoint();
    BaseJoint(const std::vector<T> &joint_item);
    BaseJoint(const std::vector<number> &, const std::vector<T> &joint_item);
    BaseJoint(std::initializer_list<T> list_values);
    // ~BaseJoint();

    /**
     * @brief Get the number of agents (i.e. the size of the joint element)
     */
    number getNumAgents() const;

    /**
     * @brief Get the element for agent i
     */
    const T &get(const number &) const;

    /**
     * @brief Get the element for agent i
     */
    T operator()(const number &);

    std::string str() const;

    /**
      * @brief print the joint item
      * @param std::ostream&
      * @param const joint<item, instance>& joint item to be printed
      * @return std::ostream&
      */
    friend std::ostream &operator<<(std::ostream &os, const BaseJoint<T> &joint)
    {
      os << joint.str();
      return os;
    }

    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &archive, const unsigned int)
    {
      archive &boost::serialization::base_object<std::vector<T>>(*this);
    }
  };
} // namespace sdm
#include <sdm/core/base_joint.tpp>
