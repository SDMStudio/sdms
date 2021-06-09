#pragma once

#include <sdm/core/base_joint.hpp>
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
  class Joint : public BaseJoint<T>
  {
  public:
    using value_type = typename BaseJoint<T>::value_type;

    Joint();
    Joint(const std::vector<T> &joint_item);
    Joint(const std::vector<number> &, const std::vector<T> &joint_item);
    Joint(std::initializer_list<T> list_values);
    // virtual ~Joint();

    // /**
    //  * @brief Get the number of agents (i.e. the size of the joint element)
    //  */
    // number getNumAgents() const;

    // /**
    //  * @brief Get the element for agent i
    //  */
    // const T &get(const number &) const;

    // /**
    //  * @brief Get the element for agent i
    //  */
    // T operator()(const number &);

    // /**
    //   * @brief print the joint item
    //   * @param std::ostream&
    //   * @param const joint<item, instance>& joint item to be printed
    //   * @return std::ostream&
    //   */
    // friend std::ostream &operator<<(std::ostream &os, const Joint<T> &j)
    // {
    //   os << "(";
    //   if (j.size() > 0)
    //   {
    //     number ag;
    //     for (ag = 0; ag < j.size() - 1; ++ag)
    //     {
    //       std::ostringstream res;
    //       res << j[ag] << ", ";
    //       sdm::tools::indentedOutput(os, res.str().c_str(), 0);
    //     }
    //     os << j[ag];
    //   }
    //   os << ")";
    //   return os;
    // }

    // friend class boost::serialization::access;

    // template <class Archive>
    // void serialize(Archive &archive, const unsigned int)
    // {
    //   archive &boost::serialization::base_object<std::vector<T>>(*this);
    // }
  };

  // // Specialisation for the Joint State 
  // template <>
  // class Joint<std::shared_ptr<State>> : public BaseJoint<std::shared_ptr<State>>, std::shared_ptr<State>
  // {
  // public :
  //   using value_type = typename BaseJoint<std::shared_ptr<State>>::value_type;

  //   Joint();
  //   Joint(const std::vector<std::shared_ptr<State>> &joint_item);
  //   Joint(const std::vector<number> &, const std::vector<std::shared_ptr<State>> &joint_item);
  //   Joint(std::initializer_list<std::shared_ptr<State>> list_values);
  // };

  // // Specialisation for the Joint Observation 
  // template <>
  // class Joint<std::shared_ptr<Observation>> : public BaseJoint<std::shared_ptr<Observation>>, std::shared_ptr<Observation>
  // {
  // public :
  //   using value_type = typename BaseJoint<std::shared_ptr<Observation>>::value_type;

  //   Joint();
  //   Joint(const std::vector<std::shared_ptr<Observation>> &joint_item);
  //   Joint(const std::vector<number> &, const std::vector<std::shared_ptr<Observation>> &joint_item);
  //   Joint(std::initializer_list<std::shared_ptr<Observation>> list_values);
  // };

} // namespace sdm
#include <sdm/core/joint.tpp>
