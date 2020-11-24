/*=============================================================================
Copyright (c) 2016 Jilles Steeve Dibangoye, Olivier Buffet, Charles Bessonet
==============================================================================*/
#pragma once

#include <vector>

#include <boost/bimap.hpp>

// #include "variations.hpp"


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
namespace sdm{
  /*!
   *  \class      joint       "joint.hpp"
   *  \brief      class of joint item instances.
   * 
   * TODO : change counter and joint_item_bimap to non-static
   */
   template<typename item, int instance>
   class joint{
   protected:
     typedef boost::bimaps::bimap<joint<item, instance>*, item> bimap;
     typedef typename bimap::value_type jitem2index;
     static bimap joint_item_bimap;
     std::vector<item> cdecisions;
     static number counter;
     agent num_agents;

   public:
     /*!
      * \fn  joint(agent, item*)
      * \brief constructor of joint item instances
      * \param agent the number of agents
      * \param item* items assigned to agents
      */
     joint(agent, const std::vector<item>&);

     ~joint();

     /*!
      * \fn  item getIndividualitem(agent)
      * \brief getter of the item assigned to an agent
      * \param agent a specified agent
      * \return item the decision assigned to agent
      */
     item getIndividualItem(agent);


     /*!
      * \fn  static item getJointItemIdx(joint<item, instance>*)
      * \brief getter of the joint item index
      * \param joint<item, instance>*
      * \return item the item index
      */
     static item getJointItemIdx(joint*);

     /*!
      * \fn  static joint* getJointItem(item)
      * \brief getter of the joint item pointer
      * \return joint* the pointer to joint item
      */
     static joint* getJointItem(item);


     /*!
      * \fn   static item getJointItemIdx(std::vector<item>const &) const
      * \brief getter of the joint item pointer
      * \return item the item associated with the joint item
      */
      static item getJointItemIdx(std::vector<item>const &);

     /*!
      * \fn std::ostream& operator<<(std::ostream&, const joint<item, instance>&)
      * \brief print the joint item
      * \param std::ostream&
      * \param const joint<item, instance>& joint item to be printed
      * \return std::ostream&
      */
     friend std::ostream& operator<<(std::ostream& os, const joint& j){
       agent ag;
       for(ag=0; ag<j.num_agents-1; ++ag){
         os << j.getIndividualItem(ag) << " ";
       } os << j.getIndividualItem(ag);
       return os;
     }
   };
}
