#pragma once

#include <vector>

#include <sdm/types.hpp>
#include <sdm/tools.hpp>
#include <sdm/core/function.hpp>
#include <sdm/core/item.hpp>
#include <sdm/utils/struct/vector.hpp>
#include <sdm/core/action/action.hpp>
#include <sdm/core/state/state.hpp>

namespace sdm
{
  /**
   * @brief This class is used for joint objects. It can be a JointHistoryTree, a JointObservation, a JointAction, etc.
   * 
   * @tparam The type of item.
   */
  template <class T>
  class Joint : public std::vector<T>, public Function<number, T>
  {
  public:
    using value_type = T;

    Joint();
    Joint(std::size_t size);
    Joint(std::size_t size, T default_value);
    Joint(const std::vector<T> &joint_item);
    Joint(const std::vector<number> &, const std::vector<T> &joint_item);
    Joint(std::initializer_list<T> list_values);
    virtual ~Joint();

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

    template <typename TOutput>
    std::shared_ptr<Joint<std::shared_ptr<TOutput>>> toJoint(); 
    
    std::string str() const;

    /**
      * @brief print the joint item
      * @param std::ostream&
      * @param const joint<item, instance>& joint item to be printed
      * @return std::ostream&
      */
    friend std::ostream &operator<<(std::ostream &os, const Joint<T> &joint_item)
    {
      os << joint_item.str();
      return os;
    }

    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &archive, const unsigned int)
    {
      archive &boost::serialization::base_object<std::vector<T>>(*this);
    }
  };

  using JointItem = Joint<std::shared_ptr<Item>>;
  using JointAction = Joint<std::shared_ptr<Action>>;
  using JointState = Joint<std::shared_ptr<State>>;
  using JointObservation = Joint<std::shared_ptr<Observation>>;

} // namespace sdm
#include <sdm/core/joint.tpp>
