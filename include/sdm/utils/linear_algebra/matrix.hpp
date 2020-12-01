/*=============================================================================
Copyright (c) 2016 Jilles Steeve Dibangoye, Olivier Buffet, Charles Bessonet
==============================================================================*/
#pragma once

#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <cmath>

#include <sdm/utils/linear_algebra/vector.hpp>

//!
//! \file     matrix.hpp
//! \author   Jilles S. Dibangoye
//! \brief    matrix class
//! \version  1.0
//! \date     10 Avril 2016
//!
//! This class provides basic methods for matrix operations.
//!

//! \namespace  sdm
//!
//! Namespace grouping all tools required for sequential decision making.
namespace sdm{

  template<typename mtype, typename vtype, typename value>
  class matrix {
  public:

    //! \fn matrix()
    //! \brief This constructor allocates an uninitialized matrix that holds zero elements.
    matrix();

    //! \fn matrix(size_t, size_t)
    //! \brief This constructor Allocates an uninitialized matrix that holds \@param1 rows of \@param2 elements.
    //! \param size_t
    //! \param size_t
    matrix(size_t, size_t);

    //! \fn ~matrix()
    //! \brief This destructor destroies the matrix.
    ~matrix();

    matrix transpose() const;

    //! \fn void resize(size_t, size_t)
    //! \brief Reallocates a matrix to hold \@param1 rows of \@param2 elements. The existing elements of the matrix
    //!        are preseved when specified.
    //! \param size_t
    //! \param size_t
    void resize(size_t, size_t);

    //! \fn value& operator()(size_t, size_t)
    //! \brief Returns a reference of the \@param1-th element in the \@param2-th row.
    //! \param  size_t
    //! \param  size_t
    //! \return value&
    value& operator()(size_t, size_t);

    //! \fn value operator()(std::size_t, std::size_t) const
    //! \brief Returns a value of the \@param1-th element in the \@param2-th row.
    //! \param  size_t
    //! \param  size_t
    //! \return value
    value operator()(size_t, size_t) const;

    //! \fn size_t rows()
    //! \brief Returns the size_t of rows.
    //! \return size_t
    size_t rows();

    //! \fn vector<vtype, value> rows(size_t)
    //! \brief Returns the row at a specified index.
    //! \return vector<vtype, value> : the row vector.
    vector<vtype, value> row(size_t);

    //! \fn size_t cols()
    //! \brief Returns the size_t of cols.
    //! \return size_t
    size_t cols();

    //! \fn void compress()
    //! \brief Compress the matrix if possible.
    void compress();


    friend std::ostream& operator<<(std::ostream& os, const matrix& arg){
      return os << arg.container;
    }

    //! \fn friend const matrix& operator+=(matrix& arg1, const matrix& arg2)
    //! \brief Returns the sum of two  matrices
    //! \param matrix&
    //! \param matrix&
    //! \return matrix
    friend const matrix& operator+=(matrix& arg1, const matrix& arg2){
      arg1.container += arg2.container;
      return arg1;
    }

    //! TODO Make this attribute private, be aware that it might have been used somewhere else
    //! as a public attribute.
    mtype container;
  };

  //! \fn  vector<vtype, value> operator*(const matrix<mtype, vtype, value>&, const vector<vtype, value>&)
  //! \brief Returns a vector that results from the product of a matrix with a vector.
  //! \param  const matrix<mtype, vtype, value>&
  //! \param  const vector<vtype, value>&
  //! \return vector<vtype, value>
  template<typename mtype, typename vtype, typename value>
  vector<vtype, value> operator*(const matrix<mtype, vtype, value>&, const vector<vtype, value>&);

  //! \fn  matrix<mtype, vtype, value> operator*(const matrix<mtype, vtype, value>&, const matrix<mtype, vtype, value>&)
  //! \brief Returns the product of two matrices.
  //! \param  const matrix<mtype, vtype, value>&
  //! \param  const matrix<mtype, vtype, value>&
  //! \return matrix<mtype, vtype, value>
  template<typename mtype, typename vtype, typename value>
  matrix<mtype, vtype, value> operator*(const matrix<mtype, vtype, value>&, const matrix<mtype, vtype, value>&);

  //! \fn  bool operator==(const matrix<mtype, vtype, value>&, const matrix<mtype, vtype, value>&)
  //! \brief Returns true if the two matrices are equals and false otherwise.
  //! \param  const matrix<mtype, vtype, value>&
  //! \param  const matrix<mtype, vtype, value>&
  //! \return bool
  template<typename mtype, typename vtype, typename value>
  bool operator==(const matrix<mtype, vtype, value>&, const matrix<mtype, vtype, value>&);


  //! \fn  matrix<mtype, vtype, value> operator^(const vector<vtype, value>&, const matrix<mtype, vtype, value>&)
  //! \brief Returns a matrix that results from the point-to-point product of a vector with a matrix.
  //! \param  const vector<vtype, value>&
  //! \param  const matrix<mtype, vtype, value>&
  //! \return matrix<mtype, vtype, value>
  template<typename mtype, typename vtype, typename value>
  matrix<mtype, vtype, value> operator^(const vector<vtype, value>&, const matrix<mtype, vtype, value>&);


  #ifdef EIGEN
    #ifdef DENSE
      using Matrix = class matrix<Eigen::MatrixXd, Eigen::VectorXd, double>;
    #endif
    #ifdef SPARSE
      using Matrix = class matrix<Eigen::SparseMatrix<double, Eigen::RowMajor>, Eigen::SparseVector<double>, double>;
    #endif
  #endif

  #ifdef BOOST
    #ifdef DENSE
      using Matrix = class matrix<boost::numeric::ublas::matrix<double>, boost::numeric::ublas::vector<double>, double>;
    #endif
    #ifdef SPARSE
      using Matrix = class matrix<boost::numeric::ublas::mapped_matrix<double>, boost::numeric::ublas::vector<double>, double>;
    #endif
  #endif

}
