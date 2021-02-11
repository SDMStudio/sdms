/*=============================================================================
Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <vector>

#include <sdm/types.hpp>

//!
//! \file     Variations.hpp
//! \author   Jilles S. Dibangoye
//! \brief    Variations class
//! \version  1.0
//! \date     12 Avril 2016
//!
//! This class provides getter and setter methods for generating Variations
//! for all kind of data including: joint actions, joint observations,
//! joint decision rules, policy trees, etc.
//!

/*!
*  \namespace  sdm
*  namespace   grouping all tools required for sequential decision making.
*/
namespace sdm
{
  /*!
  *  \class      variation      variation.hpp
  *  \typename   output         type of the output variation.
  *  \brief      iterator of Variations.
  *
  * The public interface of output suggests that it possesses a construtor \@output(arg1, arg3).
  * In which, \a arg1 is the history keys and \a arg3 the actual decisions taken.
  */
  template <typename TFunction>
  class Variations
  {
  protected:
    using TKey = typename TFunction::input_type;
    using TItem = typename TFunction::output_type;

    using iterator_t = typename std::vector<TItem>::iterator;
    using super_iterator = std::vector<iterator_t>;

    super_iterator current; //<! current element

    std::vector<TKey> p_keys_;                 //<! histories to prescribe an action
    std::vector<std::vector<TItem>> p_values_; //<! table of numbers of actions available per agent

    std::shared_ptr<TFunction> vout;
    std::shared_ptr<TFunction> vin;

    int dimension;

  public:
    /*!
    *  \fn     Variations()
    *  \brief  constructor
    *
    *  The default constructor.
    */
    Variations();

    Variations(const std::vector<std::vector<TItem>> &possible_values);

    /*!
    *  \fn     variation(const key&, const std::vector<action>&)
    *  \param  const key&   information, i.e., the set of keys to prescribe actions
    *  \param  const std::vector<action>& is the number of actions per agent
    *  \brief  constructor
    *
    *  This constructor builds a default with the number of actions per agent.
    *  Notice that \@key implements \a size_t size();
    */
    Variations(const std::vector<TKey> &possible_keys, const std::vector<std::vector<TItem>> &possible_values);

    /*!
    *  \fn     void setVariation(const key&, const std::vector<action>&)
    *  \param  const key&   information, i.e., the set of keys to prescribe actions
    *  \param  const std::vector<action>& is the number of actions per agent
    *  \brief  Sets attributes of the variation
    */
    void setVariation(const std::vector<TKey> &possible_keys, const std::vector<std::vector<TItem>> &possible_values);

    /*!
    *  \fn     ~variation()
    *  \brief  destructor
    */
    std::shared_ptr<TFunction> make_output();

    /*!
    *  \fn     const output*& begin() const
    *  \return const output*& generated variation.
    *  \brief  points to the initial variation.
    */
    std::shared_ptr<TFunction> begin();

    /*!
    *  \fn     const std::shared_ptr<TFunction>& next()
    *  \return const std::shared_ptr<TFunction>& generated variation.
    *  \brief  Returns the next variation.
    */
    std::shared_ptr<TFunction> next();

    /*!
    *  \fn     const std::shared_ptr<TFunction>& end() const
    *  \return const std::shared_ptr<TFunction>& generated variation.
    *  \brief  points to the final variation.
    */
    std::shared_ptr<TFunction> end();
  };
} // namespace sdm
#include <sdm/utils/decision_rules/variations.tpp>