/**
 * @file reward.hpp
 * @author Jilles S. Dibangoye
 * @brief This file contains the implementation of the Reward model.
 * @version 1.0
 * @date 09/03/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <vector>

#include <sdm/types.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/utils/linear_algebra/mapped_matrix.hpp>
#include <sdm/utils/linear_algebra/sdms_vector.hpp>

namespace sdm
{

  // template <typename TState, typename TAction, template <typename TI, typename TV> class TMatrix = MappedMatrix>
  // class BaseReward
  // {
  // public:
  //   using value_type = double;

  //   BaseReward();
  //   BaseReward(number, number);
  //   BaseReward(Reward &copy);

  //   void initReward(number, number);

  //   double getReward(const TState &, const TAction &) const;
  //   const TMatrix<TState, TAction>::vector_type &getReward(const TAction &) const;
  //   const TMatrix<TState, TAction> &getReward() const;

  //   void setReward(const TState &, const TAction &, double);
  //   void setReward(const TAction&, TMatrix<TState, TAction>::vector_type);

  //   double getMaxReward() const;
  //   double getMinReward() const;

  //   friend std::ostream &operator<<(std::ostream &os, const Reward &reward_fct)
  //   {

  //     number ja;
  //     os << "<reward> " << std::endl;
  //     for (ja = 0; ja < reward_fct.rewards.size(); ++ja)
  //     {
  //       os << "\t" << reward_fct.getReward(ja) << "" << std::endl;
  //     }
  //     os << "</reward> " << std::endl;
  //     return os;
  //   }

  // protected:
  //   //! \brief model of the outcomes of the entire team of collaborative agents involved in the system.
  //   TMatrix<TState, TAction> rewards;

  //   double max = -999999, min = +999999;
  // };

  /**
   * 
   * @brief This class provides getter and setter methods for the reward model.
   * 
   */
  class Reward
  {
  public:
    using value_type = double;
    using vector_type = DenseVector<number, value_type>;

    Reward();

    //! \fn       Reward(number num_jactions, number num_states)
    //! \param    num_jactions Number of joint actions
    //! \param    num_states Number of states
    //! \brief    Reward constructor
    Reward(number, number);

    Reward(Reward &copy);

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
    const DenseVector<number> &getReward(number) const;

    //! \fn       const std::vector<DenseVector<number>>& getReward() const
    //! \brief    Returns list of reward vectors
    //! \return   the entire reward function
    const std::vector<DenseVector<number>> &getReward() const;

    //! \fn       void setReward(action, const vector&)
    //! \param    a A specific joint action
    //! \param    v the vector of rewards for corresponding action
    //! \brief    Sets the reward vector for action
    void setReward(number, DenseVector<number>);

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

      number ja;
      os << "<reward> " << std::endl;
      for (ja = 0; ja < reward_fct.rewards.size(); ++ja)
      {
        os << "\t" << reward_fct.getReward(ja) << "" << std::endl;
      }
      os << "</reward> " << std::endl;
      return os;
    }

  protected:
    //! \brief model of the outcomes of the entire team of collaborative agents involved in the system.
    std::vector<vector_type> rewards;

    double max = -999999, min = +999999;
  };

}
