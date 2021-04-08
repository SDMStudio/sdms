/**
 * @file vector_impl.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief File for VectorImpl interface.
 * @version 0.1
 * @date 12/01/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <cstdlib>
#include <sstream>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
  /**
   * @brief The vector interface. To be considered as a vector in SDM'Studio, a class must implement all the following functions. 
   * 
   * @tparam I Type of the index.
   * @tparam T Type of the values.
   */
  template <typename... I>
  class TensorImpl
  {
  public:
    virtual std::string str() const = 0;
  };

} // namespace sdm
