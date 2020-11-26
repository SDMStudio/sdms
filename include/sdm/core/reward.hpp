/*=============================================================================
  Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <vector>

#include <sdm/types.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>

//!
//! \file     reward.hpp
//! \author   Jilles S. Dibangoye
//! \brief    reward class
//! \version  1.0
//! \date     12 Avril 2016
//!
//! This class provides getter and setter methods for the reward model.
//!

//! \namespace  sdm
//!
//! Namespace grouping all tools required for sequential decision making.
namespace sdm{

  //!
  //! \class  reward  reward.hpp
  //!
  class Reward {
    protected:
      //! \brief model of the outcomes of the entire team of collaborative agents involved in the system.
      std::vector<Vector> rewards;

      double max = -999999, min = +999999;

    public:

      Reward();

      //! \fn       Reward(number num_jactions, number num_states)
      //! \param    num_jactions Number of joint actions
      //! \param    num_states Number of states
      //! \brief    Reward constructor
      Reward(number, number);

      //! \fn       void initReward(number num_jactions, number num_states)
      //! \param    num_jactions Number of joint actions
      //! \param    num_states Number of states
      //! \brief    Inits the reward model
      void initReward(number, number);

      //! \fn       value getReward(number s, number a) const
      //! \param    s A specific state
      //! \param    a A specific joint action
      //! \brief    Returns reward
      //! \return   reward at state s and jaction a
      double getReward(number, number) const;

      //! \fn       value setReward(number s, number a) const
      //! \param    s A specific state
      //! \param    a A specific joint action
      //! \param    value The corresponding reward
      //! \brief    Reward setter
      void setReward(number, number, double);

      //! \fn       const vector& getReward(action) const
      //! \param    a A specific joint action
      //! \brief    Get reward vector for given action
      //! \return   the vector of rewards for given action
      const Vector& getReward(number) const;

      //! \fn       const std::vector<Vector>& getReward() const
      //! \brief    Returns list of reward vectors
      //! \return   the entire reward function
      const std::vector<Vector>& getReward() const;

      //! \fn       void setReward(action, const vector&)
      //! \param    a A specific joint action
      //! \param    v the vector of rewards for corresponding action 
      //! \brief    Sets the reward vector for action
      void setReward(number, const Vector&);

      double getMaxReward() const;

      double getMinReward() const;

      /*!
    * \fn std::ostream& operator<<(std::ostream&, const dpomdp&)
    * \brief print the dpomdp
    * \param std::ostream&
    * \param const dpomdp& the individual decision rule to be printed
    * \return std::ostream&
    *
    * This method should produce an output of this form:
    */
    friend std::ostream &operator<<(std::ostream &os, const Reward &reward_fct)
    {
      
      action ja;
      os << "\033[1m\033[32m ######################## \t\t<reward> \033[0m" << std::endl;
      for (ja = 0; ja < reward_fct.rewards.size(); ++ja)
      {
        os << "\033[1m\033[32m #" << reward_fct.getReward(ja) << "\033[0m" << std::endl;
      }
      os << "\033[1m\033[32m ######################## \t\t</reward> \033[0m" << std::endl;
      return os;
    }
  };

}
