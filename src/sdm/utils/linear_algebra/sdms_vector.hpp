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

#include <vector>
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
  public:
    sdmsVector();
    sdmsVector(sdm::size_t);
    sdmsVector(sdm::size_t, const T &);

    template <class AE>
    sdmsVector(const boost::numeric::ublas::vector_expression<AE> &ae);
    sdmsVector(const sdmsVector &);
    sdmsVector(const std::vector<T> &);

    void init(T);
    T at(const I &) const;

    T sum();
    T norm_1() const;
    T norm_2() const;

    T min();
    I argmin();
    T max();
    I argmax();

    sdmsVector transpose() const;

    /**
     * @brief Compare two vectors. Return true if all values are lower or equal to the second vector.
     * 
     * @return true 
     * @return false 
     */
    bool operator<=(const sdmsVector &) const;
    bool operator==(const sdmsVector &) const;
    bool operator!=(const sdmsVector &) const;

    template <class AE>
    T dot(const boost::numeric::ublas::vector_expression<AE> &) const;

    template <class AE>
    T operator^(const boost::numeric::ublas::vector_expression<AE> &ae);

    std::string str() const;

  protected:
    std::pair<I, T> getMin() const;
    std::pair<I, T> getMax() const;

    template <class Archive>
    void serialize(Archive &archive, const unsigned int);
  };

  /**
   * @brief Sparse vector are vectors that store only non-zero values.
   * 
   * @tparam I Type of index
   * @tparam T Type of value
   */
  template <typename I, typename T = double>
  using SparseVector = sdmsVector<I, T, boost::numeric::ublas::mapped_vector<T>>;

  /**
   * @brief Dense vector are standard vector.
   * 
   * @tparam I Type of index
   * @tparam T Type of value
   */
  template <typename I, typename T = double>
  using DenseVector = sdmsVector<I, T, boost::numeric::ublas::vector<T>>;

} // namespace sdm

#include <sdm/utils/linear_algebra/sdms_vector.tpp>
