/**
 * @file state.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief The file for state class
 * @version 0.1
 * @date 11/12/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#pragma once

#include <sdm/core/item.hpp>


//! \namespace  sdm
//!
//! Namespace grouping all tools required for sequential decision making.
namespace sdm
{
  template <class item_t>
  struct State : public Item<item_t>
  {
        virtual std::string str() const = 0;
  };
} // namespace sdm