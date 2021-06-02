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
#include <boost/numeric/ublas/vector_sparse.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <assert.h>

#include <sdm/utils/linear_algebra/vector_impl.hpp>

namespace sdm
{
  /**
   * @brief Create a SDMS Vector. A SMDS Vector is used to optimize the calculation, however, you have to be careful when using it because it's not possible to add element after the initialization
   * 
   * @tparam I Type of index
   * @tparam T Type of value
   */
  template <class I, class T, class TBaseVector>
  class sdmsVector : public VectorImpl<I, T>
  {
  public:
    // using array_type = typename TBaseVector::array_type;
    // using value_type = typename array_type::value_type;

    static double PRECISION;

    sdmsVector();

    /**
     * @brief Create a SDMS Vector. In order to create a vector, it's necessary to provide a map that associated the I element with a specific index and a specific Value.
     * 
     * @param std::shared_ptr<std::unordered_map<I, size_t>> : A map that associate the element I with a specific index
     * @param std::shared_ptr<std::unordered_map<I, T>> : A map that associate the element I with a specific value
     * @param TypeVector : The type of Vector. The only possibility is Dense Vector or Sparse Vector
     * 
     */
    sdmsVector(std::shared_ptr<std::unordered_map<I, size_t>>, std::shared_ptr<std::unordered_map<I, T>>);

    virtual ~sdmsVector() {}

    T at(const I &) const;
    T getValueAt(const I &) const;
    void setValueAt(const I &, const T &value);

    T sum();
    T norm_1() const;
    T norm_2() const;

    T min();
    I argmin();
    T max();
    I argmax();

    // sdmsVector transpose() const;

    const std::vector<I> &getIndexes() const;

    /**
     * @brief Compare two vectors. Return true if all values are lower or equal to the second vector.
     * 
     * @return true 
     * @return false 
     */
    bool operator<=(const sdmsVector &) const;
    bool operator==(const sdmsVector &) const;
    bool operator!=(const sdmsVector &) const;
    bool is_equal(const sdmsVector &other, double precision) const;

    // template <class AE>
    T dot(const sdmsVector &v2) const;

    // template <class AE>
    T operator^(const sdmsVector &v2) const;

    std::string str() const;

    auto begin() const { return this->iterator_.begin(); }
    auto end() const { return this->iterator_.end(); }

    template <class Archive>
    void serialize(Archive &archive, const unsigned int);

    size_t size() const;
    std::shared_ptr<std::unordered_map<I, size_t>> getMapElementToIndex() const;

  protected:
    std::vector<I> vector_element_;
    // std::shared_ptr<std::unordered_map<I, size_t>> map_index_to_value_;
    std::shared_ptr<std::unordered_map<I, size_t>> map_element_to_index_;

    // std::map<I, T> iterator_;

    TBaseVector tbasevector_;

    std::pair<I, T> getMin() const;
    std::pair<I, T> getMax() const;

    // friend sdmsVector operator*(const T &arg1, const sdmsVector &arg2)
    // {
    //   sdmsVector vnew = arg1 * static_cast<TBaseVector>(arg2);
    //   vnew.setIndexes(arg2.getIndexes());
    //   return vnew;
    // }
  };

  /**
   * @brief Sparse vector are vectors that store only non-zero values.
   * 
   * @tparam I Type of index
   * @tparam T Type of value
   */
  template <typename I = size_t, typename T = double>
  using SparseVector = sdmsVector<I, T, boost::numeric::ublas::mapped_vector<T>>;

  /**
   * @brief Dense vector are standard vector.
   * 
   * @tparam I Type of index
   * @tparam T Type of value
   */
  template <typename I = size_t, typename T = double>
  using DenseVector = sdmsVector<I, T, boost::numeric::ublas::vector<T>>;

} // namespace sdm

#include <sdm/utils/linear_algebra/sdms_vector.tpp>
