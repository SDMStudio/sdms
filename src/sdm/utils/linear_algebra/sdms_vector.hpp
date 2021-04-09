/**
 * @file sdms_vector.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief 
 * @version 0.1
 * @date 07/01/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <assert.h>

#include <sdm/utils/linear_algebra/vector_impl.hpp>

namespace sdm
{
  /**
   * @brief Vector that implement the sdm::VectorImpl interface. 
   * 
   * @tparam I Type of index
   * @tparam T Type of value
   * @tparam TBaseVector Type of the base structure
   */
  template <class I, class T, class TBaseVector>
  class sdmsVector : public TBaseVector, public VectorImpl<I, T>
  {
  protected:
    std::pair<I, T> getMin() const;
    std::pair<I, T> getMax() const;

  public:
    sdmsVector();
    sdmsVector(I);
    sdmsVector(I, T);
    sdmsVector(const sdmsVector &);

    void init(T);
    T at(const I &) const;

    T sum();
    T norm_1() const;
    T norm_2() const;

    T min() const;
    I argmin() const;
    T max() const;
    I argmax() const;

    bool operator<(const sdmsVector &) const;
    T operator^(const sdmsVector &) const;
    T dot(const sdmsVector &) const;

    bool operator==(const sdmsVector &) const;
    bool operator!=(const sdmsVector &) const;

    std::string str() const;
  };

  /**
   * @brief Sparse vector are vectors that store only non-zero values.
   * 
   * @tparam I Type of index
   * @tparam T Type of value
   */
  template <typename I, typename T>
  using SparseVector = sdmsVector<I, T, boost::numeric::ublas::mapped_vector<T>>;

  /**
   * @brief Dense vector are standard vector.
   * 
   * @tparam I Type of index
   * @tparam T Type of value
   */
  template <typename I, typename T>
  using DenseVector = sdmsVector<I, T, boost::numeric::ublas::vector<T>>;

} // namespace sdm

#include <sdm/utils/linear_algebra/sdms_vector.tpp>
