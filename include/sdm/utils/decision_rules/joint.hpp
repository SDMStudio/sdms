/*=============================================================================
Copyright (c) 2016 Jilles Steeve Dibangoye, Olivier Buffet, Charles Bessonet
==============================================================================*/
#pragma once

#include <vector>

#include <sdm/types.hpp>

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
  class Joint : public std::vector<item>
  {
  protected:
    //! \brief the joint item
    // std::vector<item> joint_items_;

    //! \brief the number of agents
    number num_agents_;

  public:
    Joint(const std::vector<item> &joint_item) : std::vector<item>(joint_item), num_agents_(joint_item.size())
    {
    }

    Joint(const std::vector<number> &num_agents, const std::vector<item> &joint_item) : std::vector<item>(joint_item), num_agents_(num_agents.size()) {}

    /*!
      * \fn std::ostream& operator<<(std::ostream&, const joint<item, instance>&)
      * \brief print the joint item
      * \param std::ostream&
      * \param const joint<item, instance>& joint item to be printed
      * \return std::ostream&
      */
    friend std::ostream &operator<<(std::ostream &os, const Joint &j)
    {
      number ag;
      for (ag = 0; ag < j.size() - 1; ++ag)
      {
        os << j[ag] << " ";
      }
      os << j[ag];
      return os;
    }
  };

  template class Joint<number>;

  typedef Joint<number> JointItem;
} // namespace sdm
