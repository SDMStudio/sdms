/**
 * @file hsvi.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief HSVI algorithm
 * @version 0.1
 * @date 22/12/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#pragma once

#include <sdm/types.hpp>

namespace sdm
{
  template <typename TSupport>
  class MappedDisreteDistrib : public MappedVector<TSupport, double>
  {
  public:
    MappedDisreteDistrib();
    TSupport sample();
  };
} // namespace sdm
// #include <sdm/core/mapped_distrib.tpp>
