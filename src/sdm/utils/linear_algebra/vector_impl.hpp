
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
  template <typename I, typename T>
  class VectorImpl
  {
  public:
    virtual T norm_1() const = 0;
    virtual T norm_2() const = 0;

    virtual T min() = 0;
    virtual I argmin() = 0;

    virtual T max() = 0;
    virtual I argmax() = 0;
    virtual T at(const I &) const = 0;

    virtual std::string str() const = 0;

    // T operator[](const I &) const;

    // virtual T operator^(const VectorImpl &) const = 0;
    // virtual T dot(const VectorImpl &) const = 0;
  };

} // namespace sdm
