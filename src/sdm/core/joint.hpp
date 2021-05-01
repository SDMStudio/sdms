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
  template <typename item>
  class Joint : public std::vector<item>, public Function<number, item>
  {
  protected:
    //! \brief the number of agents
    number num_agents_ = 0;

  public:
    using value_type = item;

    Joint() : std::vector<item>() {}
    Joint(const std::vector<item> &joint_item) : std::vector<item>(joint_item), num_agents_(joint_item.size()) {}
    Joint(const std::vector<number> &, const std::vector<item> &joint_item) : std::vector<item>(joint_item), num_agents_(joint_item.size()) {}
    virtual ~Joint() {}

    number getNumAgents() const
    {
      return this->size();
    }

    /*!
      * \fn std::ostream& operator<<(std::ostream&, const joint<item, instance>&)
      * \brief print the joint item
      * \param std::ostream&
      * \param const joint<item, instance>& joint item to be printed
      * \return std::ostream&
      */
    friend std::ostream &operator<<(std::ostream &os, const Joint<item> &j)
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

    item operator()(const number &i)
    {
      return (*this)[i];
    }
  };

  template class Joint<number>;

  typedef Joint<number> JointItem;
} // namespace sdm

namespace std
{
  template <typename T>
  struct hash<sdm::Joint<T>>
  {
    typedef sdm::Joint<T> argument_type;
    typedef std::size_t result_type;
    result_type operator()(argument_type const &in) const
    {
      size_t size = in.size();
      size_t seed = 0;
      for (size_t i = 0; i < size; i++)
        //Combine the hash of the current vector with the hashes of the previous ones
        sdm::hash_combine(seed, in[i]);
      return seed;
    }
  };
}
