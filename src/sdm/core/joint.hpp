#pragma once

#include <vector>

#include <sdm/types.hpp>
#include <sdm/tools.hpp>
#include <sdm/core/function.hpp>
#include <sdm/utils/struct/vector.hpp>

namespace sdm
{
  /**
   * @brief This class is used for joint objects. It can be a JointHistoryTree, a JointObservation, a JointAction, etc.
   * 
   * @tparam The type of item.
   */
  template <typename T>
  class Joint : public std::vector<T>, public Function<number, T>
  {
  public:
    using value_type = T;

    Joint();
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

    /**
      * @brief print the joint item
      * @param std::ostream&
      * @param const joint<item, instance>& joint item to be printed
      * @return std::ostream&
      */
    friend std::ostream &operator<<(std::ostream &os, const Joint<T> &j)
    {
      os << "(";
      if (j.size() > 0)
      {
        number ag;
        for (ag = 0; ag < j.size() - 1; ++ag)
        {
          std::ostringstream res;
          res << j[ag] << ", ";
          sdm::tools::indentedOutput(os, res.str().c_str(), 0);
        }
        os << j[ag];
      }
      os << ")";
      return os;
    }

    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &archive, const unsigned int)
    {
      archive &boost::serialization::base_object<std::vector<T>>(*this);
    }
  };

  template class Joint<number>;

  typedef Joint<number> JointItem;
} // namespace sdm

#include <sdm/core/joint.tpp>