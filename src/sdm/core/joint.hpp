/*=============================================================================
Copyright (c) 2016 Jilles Steeve Dibangoye, Olivier Buffet, Charles Bessonet
==============================================================================*/
#pragma once

#include <vector>

#include <sdm/types.hpp>
#include <sdm/tools.hpp>
#include <sdm/core/function.hpp>

//!
//! \file     joint.hpp
//! \author   Jilles S. Dibangoye
//! \brief    joint class
//! \version  1.0
//! \date     12 Avril 2016
//!
//! This class provides getter and setter methods for joint objects.
//!

/*!
 *  \namespace  sdm
 *  namespace   grouping all tools required for sequential decision making.
 */
namespace sdm
{
  /*!
   *  \class      Joint
   *  \brief      class of joint item instances.
   * 
   */
  template <typename item>
  class Joint : public std::vector<item>, public Function<number, item>
  {
  protected:
    //! \brief the number of agents
    number num_agents_;

  public:
    using value_type = item;

    Joint() : std::vector<item>() {}
    Joint(const std::vector<item> &joint_item) : std::vector<item>(joint_item), num_agents_(joint_item.size()) {}
    Joint(const std::vector<number> &num_agents, const std::vector<item> &joint_item) : std::vector<item>(joint_item), num_agents_(joint_item.size()) {}

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
          sdm::tools::indentedOutput(os, res.str().c_str());
          // os << j[ag] << ", ";
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

  //using boost::hash_combine
  template <class T>
  inline void hash_combine(std::size_t &seed, T const &v)
  {
    seed ^= std::hash<T>()(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  }
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
