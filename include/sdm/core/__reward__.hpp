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
  class __reward__ {
    protected:
      //! \brief model of the outcomes of the entire team of collaborative agents involved in the system.
      std::vector<Vector> rewards;

      double max = -999999, min = +999999;

    public:

      //! \fn       void initReward(state, action)
      //! \param    action
      //! \param    state
      //! \brief    Inits the reward model
      void initReward(action, state);

      //! \fn       value getReward(state, action) const
      //! \param    state
      //! \param    action
      //! \brief    Returns reward
      //! \return   value
      double getReward(state, action) const;

      //! \fn       value getReward(state, action) const
      //! \param    state
      //! \param    action
      //! \param    value
      //! \brief    Returns reward
      void setReward(state, action, double);

      //! \fn       const vector& getReward(action) const
      //! \param    action
      //! \brief    Returns reward vector
      //! \return   const vector&
      const Vector& getReward(action) const;

      //! \fn       const std::vector<Vector>& getReward() const
      //! \brief    Returns list of reward vectors
      //! \return   const std::vector<Vector>&
      const std::vector<Vector>& getReward() const;

      //! \fn       void setReward(action, const vector&)
      //! \param    action
      //! \param    const vector&
      //! \brief    Sets the reward vector for action
      void setReward(action, const Vector&);

      double getMaxReward();

      double getMinReward();

      /*!
    * \fn std::ostream& operator<<(std::ostream&, const dpomdp&)
    * \brief print the dpomdp
    * \param std::ostream&
    * \param const dpomdp& the individual decision rule to be printed
    * \return std::ostream&
    *
    * This method should produce an output of this form:
    */
    friend std::ostream &operator<<(std::ostream &os, const __reward__ &reward_fct)
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
