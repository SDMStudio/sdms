/**
 * @file space.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief abstract space class
 * @version 0.1
 * @date 17/12/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 * This is an abstract interface for Spaces.
 * 
 */
#pragma once

#include <vector>
#include <boost/bimap.hpp>
#include <sdm/types.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
  /**
   * @class Space
   * @brief Abstract space class 
   */
  class Space
  {
  public:
    virtual std::vector<number> getDim() const = 0;
    virtual std::string str() const = 0;
  };
} // namespace sdm
