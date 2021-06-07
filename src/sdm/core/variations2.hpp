/**
 * @file variations.hpp
 * @author Jilles S. Dibangoye
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief This class provides getter and setter methods for generating Variations2 for all kind of data including: joint actions, joint observations, joint decision rules, policy trees, etc.
 * @version 1.0
 * @date 09/03/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <vector>

#include <sdm/types.hpp>

namespace sdm
{

  /**
  * @brief Iterator of Variations2.
  * 
  * @tparam TFunction type of the output function.
  * 
  * The public interface of output suggests that it possesses a construtor \@output(arg1, arg3).
  * In which, \a arg1 is the history keys and \a arg3 the actual decisions taken.
  */
  template <typename TFunction>
  class Variations2
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
    /**
    *  @fn     Variations2()
    *  @brief  constructor
    *
    *  The default constructor.
    */
    Variations2();

    Variations2(const std::vector<std::vector<TItem>> &possible_values);

    /**
    *  @param  const key&   information, i.e., the set of keys to prescribe actions
    *  @param  const std::vector<action>& is the number of actions per agent
    *  @brief  This constructor builds a default with the number of actions per agent.
    *  Notice that \@key implements \a size_t size();
    * 
    */
    Variations2(const std::vector<TKey> &possible_keys, const std::vector<std::vector<TItem>> &possible_values);

    /**
    *  @param  const key&   information, i.e., the set of keys to prescribe actions
    *  @param  const std::vector<action>& is the number of actions per agent
    *  @brief  Sets attributes of the variation
    */
    void setVariation(const std::vector<TKey> &possible_keys, const std::vector<std::vector<TItem>> &possible_values);

    /**
    *  @brief  destructor
    */
    std::shared_ptr<TFunction> make_output();

    /**
    *  @brief  Pointer on the initial variation.
    *  @return generated variation.
    */
    std::shared_ptr<TFunction> begin();

    /**
    *  @return generated variation.
    *  @brief  Returns the next variation.
    */
    std::shared_ptr<TFunction> next();

    /**
    *  @brief  Pointer on the final variation.
    *  @return generated variation.
    */
    std::shared_ptr<TFunction> end();
  };
} // namespace sdm
#include <sdm/core/variations2.tpp>