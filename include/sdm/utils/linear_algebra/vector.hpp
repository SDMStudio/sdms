/*=============================================================================
Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Dense>

#include <assert.h>

#include <sdm/types.hpp>


//!
//! \file     vector.hpp
//! \author   Jilles S. Dibangoye
//! \brief    vector class
//! \version  1.0
//! \date     10 Avril 2016
//!
//! This class provides basic methods for vector operations.
//!

//! \namespace  sdm
//!
//! Namespace grouping all tools required for sequential decision making.
namespace sdm{

  template<typename type, typename value>
  class vector{
  protected:
    number _size_;

    //! TODO Make this attribute private, be aware that it might have been used somewhere else
    //! as a public attribute.
    type container;

  public:
    //! \fn vector()
    //! \brief This constructor allocates an uninitialized vector that holds zero elements.
    vector();

    //! \fn vector(number)
    //! \brief This constructor allocates an uninitialized vector that holds "number" elements.
    //! \param number
    vector(number);

    //! \fn vector(const vector&)
    //! \brief This is a copy constructor.
    //! \param const vector&
    vector(const vector&);

    //! \fn ~vector()
    //! \brief This destructor destroies the vector.
    ~vector();

    //! \fn value sum()
    //! \brief Returns the sum of all elements in the vector
    //! \param value
    value sum();


    //! \fn value norm_2()
    //! \brief Returns the norm_2 of a vector
    //! \param value
    value norm_2() const;

    //! \fn value norm_1()
    //! \brief Returns the norm_1 of a vector
    //! \param value
    value norm_1() const;

    //! \fn value norm_sawtooth()
    //! \brief Returns the norm_sawtooth of a vector
    //! \param value
    value norm_sawtooth(const vector&) const;


    //! \brief Returns the min of all elements in the vector
    //! \param value
    value min() const;


    //! \fn value max()
    //! \brief Returns the max of all elements in the vector
    //! \param value
    value max() const;


    number size() const;

    //! \fn void resize(number)
    //! \brief Reallocates a vector to hold "number" elements.
    //! \param number
    void resize(number);

    //! \fn
    //! \brief Initialize the vector
    //! \param value : default value for all entries
    void init(value);

    vector transpose() const;

    const type& getContainer() const;

    void setContainer(const type&);

    //! \fn value& operator[](number)
    //! \brief Returns a reference of the i-th element.
    //! \param number
    //! \return value&
    value& operator[](number);

    //! \fn value operator[](number) const
    //! \brief Returns a value of the i-th element.
    //! \param number
    //! \return value
    value operator[](number) const;

    //! \fn friend vector<type, value>& operator/=(const vector<type, value>&, const value&)
    //! \brief Returns the vector normalized by a scalar
    //! \param const vector<type, value>&
    //! \param const value&
    //! \return vector<type, value>&
    friend const vector& operator/=(vector& arg1, const value& arg2){
      assert( arg2 != 0 );
      arg1.container /= arg2;
      return arg1;
    }

    //! \fn friend bool operator*(const value&, const vector<type, value>&)
    //! \brief Returns the product of a scalar with a vector
    //! \param const value&
    //! \param const vector<type, value>&
    //! \return vector<type, value>
    friend vector operator*(const value& arg1, const vector& arg2){
        vector vnew;
        vnew.setContainer( arg1 * arg2.getContainer() );
        return vnew;
    }

    friend std::ostream& operator<<(std::ostream& os, const vector& arg){
      #ifdef EIGEN
        return os << arg.getContainer().transpose();
      #endif
      #ifdef BOOST
        return os << arg.getContainer();
      #endif
    }

    //! \fn friend vector<type, value>& operator+=(const vector<type, value>&, const vector<type, value>&)
    //! \brief Returns the sum of two  vectors
    //! \param const vector<type, value>&
    //! \param const vector<type, value>&
    //! \return vector<type, value>&
    friend const vector& operator+=(vector& arg1, const vector& arg2){
      arg1.setContainer( arg1.getContainer() + arg2.getContainer() );
      return arg1;
    }

    //! \fn friend vector<type, value>& operator+=(const vector<type, value>&, const vector<type, value>&)
    //! \brief Returns the sum of two  vectors
    //! \param const vector<type, value>&
    //! \param const vector<type, value>&
    //! \return vector<type, value>&
    friend const vector& operator-=(vector& arg1, const vector& arg2){
      arg1.setContainer( arg1.getContainer() - arg2.getContainer() );
      return arg1;
    }

    //! \fn friend value operator^(const vector<type, value>&, const vector<type, value>&)
    //! \brief Returns the scalar product of the two vectors.
    //! \param const vector<type, value>&
    //! \param const vector<type, value>&
    //! \return bool
    value operator^(const vector<type, value>&) const;

    //! \fn friend bool operator==(const vector<type, value>&, const vector<type, value>&)
    //! \brief Returns true if both vectors are equals and false otherwise.
    //! \param const vector<type, value>&
    //! \param const vector<type, value>&
    //! \return bool
    bool operator==(const vector<type, value>&) const;
  };



  #ifdef EIGEN
    #ifdef DENSE
      using Vector = class vector<Eigen::VectorXd, double>;
    #endif
    #ifdef SPARSE
      using Vector = class vector<Eigen::SparseVector<double>, double>;
    #endif
  #endif


  #ifdef BOOST
    #ifdef DENSE
      using Vector = class vector<boost::numeric::ublas::vector<double>, double>;
    #endif
    #ifdef SPARSE
      using Vector = class vector<boost::numeric::ublas::vector<double>, double>;
    #endif
  #endif

}
