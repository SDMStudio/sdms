/*=============================================================================
Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <vector>

#include <sdm/types.hpp>

//!
//! \file     variations.hpp
//! \author   Jilles S. Dibangoye
//! \brief    variations class
//! \version  1.0
//! \date     12 Avril 2016
//!
//! This class provides getter and setter methods for generating variations
//! for all kind of data including: joint actions, joint observations,
//! joint decision rules, policy trees, etc.
//!

/*!
*  \namespace  sdm
*  namespace   grouping all tools required for sequential decision making.
*/
namespace sdm{
  /*!
  *  \class      variation      variation.hpp
  *  \typename   output         type of the output variation.
  *  \brief      iterator of variations.
  *
  * The public interface of output suggests that it possesses a construtor \@output(arg1, arg3).
  * In which, \a arg1 is the history keys and \a arg3 the actual decisions taken.
  */
  template<typename key, typename output>
  class variations{
  protected:
    std::vector<action> decisions;  //<! table of numbers of actions available per agent
    action* current = nullptr;      //<! current element
    action* cbegin = nullptr;       //<! initial element
    action* cend = nullptr;         //<! final element
    key parameters;                 //<! histories to prescribe an action

    action limit();

    ptrdiff_t index();

  public:
    /*!
    *  \fn     variations()
    *  \brief  constructor
    *
    *  The default constructor.
    */
    variations();

    /*!
    *  \fn     variation(const key&, const std::vector<action>&)
    *  \param  const key&   information, i.e., the set of keys to prescribe actions
    *  \param  const std::vector<action>& is the number of actions per agent
    *  \brief  constructor
    *
    *  This constructor builds a default with the number of actions per agent.
    *  Notice that \@key implements \a size_t size();
    */
    variations(const key&, const std::vector<action>&);

    /*!
    *  \fn     void setVariation(const key&, const std::vector<action>&)
    *  \param  const key&   information, i.e., the set of keys to prescribe actions
    *  \param  const std::vector<action>& is the number of actions per agent
    *  \brief  Sets attributes of the variation
    */
    void setVariation(const key&, const std::vector<action>&);

    /*!
    *  \fn     ~variation()
    *  \brief  destructor
    */
    virtual ~variations();

    /*!
    *  \fn     const output*& begin() const
    *  \return const output*& generated variation.
    *  \brief  points to the initial variation.
    */
    output* begin();

    /*!
    *  \fn     const output*& next()
    *  \return const output*& generated variation.
    *  \brief  Returns the next variation.
    */
    output* next();

    /*!
    *  \fn     const output*& end() const
    *  \return const output*& generated variation.
    *  \brief  points to the final variation.
    */
    output* end();
  };
}
