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
namespace sdm
{

  template <typename type, typename value>
  class vector
  {
  protected:
    size_t _size_;

    //! TODO Make this attribute private, be aware that it might have been used somewhere else
    //! as a public attribute.
    type container;

  public:
    //! \fn vector()
    //! \brief This constructor allocates an uninitialized vector that holds zero elements.
    vector();

    //! \fn vector(size_t)
    //! \brief This constructor allocates an uninitialized vector that holds "size_t" elements.
    //! \param size_t
    vector(size_t);

    //! \fn vector(const vector&)
    //! \brief This is a copy constructor.
    //! \param const vector&
    vector(const vector &);

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
    value norm_sawtooth(const vector &) const;

    //! \brief Returns the min of all elements in the vector
    //! \param value
    value min() const;

    //! \fn value max()
    //! \brief Returns the max of all elements in the vector
    //! \param value
    value max() const;

    size_t size() const;

    //! \fn void resize(size_t)
    //! \brief Reallocates a vector to hold "size_t" elements.
    //! \param size_t
    void resize(size_t);

    //! \fn
    //! \brief Initialize the vector
    //! \param value : default value for all entries
    void init(value);

    vector transpose() const;

    const type &getContainer() const;

    void setContainer(const type &);

    //! \fn value& operator[](size_t)
    //! \brief Returns a reference of the i-th element.
    //! \param size_t
    //! \return value&
    value &operator[](size_t);

    //! \fn value operator[](size_t) const
    //! \brief Returns a value of the i-th element.
    //! \param size_t
    //! \return value
    value operator[](size_t) const;

    //! \fn friend vector<type, value>& operator/=(const vector<type, value>&, const value&)
    //! \brief Returns the vector normalized by a scalar
    //! \param const vector<type, value>&
    //! \param const value&
    //! \return vector<type, value>&
    friend const vector &operator/=(vector &arg1, const value &arg2)
    {
      assert(arg2 != 0);
      arg1.container /= arg2;
      return arg1;
    }

    //! \fn friend bool operator*(const value&, const vector<type, value>&)
    //! \brief Returns the product of a scalar with a vector
    //! \param const value&
    //! \param const vector<type, value>&
    //! \return vector<type, value>
    friend vector operator*(const value &arg1, const vector &arg2)
    {
      vector vnew;
      vnew.setContainer(arg1 * arg2.getContainer());
      return vnew;
    }

    friend std::ostream &operator<<(std::ostream &os, const vector &arg)
    {
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
    friend const vector &operator+=(vector &arg1, const vector &arg2)
    {
      arg1.setContainer(arg1.getContainer() + arg2.getContainer());
      return arg1;
    }

    //! \fn friend vector<type, value>& operator+=(const vector<type, value>&, const vector<type, value>&)
    //! \brief Returns the sum of two  vectors
    //! \param const vector<type, value>&
    //! \param const vector<type, value>&
    //! \return vector<type, value>&
    friend const vector &operator-=(vector &arg1, const vector &arg2)
    {
      arg1.setContainer(arg1.getContainer() - arg2.getContainer());
      return arg1;
    }

    //! \fn friend value operator^(const vector<type, value>&, const vector<type, value>&)
    //! \brief Returns the scalar product of the two vectors.
    //! \param const vector<type, value>&
    //! \param const vector<type, value>&
    //! \return bool
    value operator^(const vector<type, value> &) const;

    //! \fn friend bool operator==(const vector<type, value>&, const vector<type, value>&)
    //! \brief Returns true if both vectors are equals and false otherwise.
    //! \param const vector<type, value>&
    //! \param const vector<type, value>&
    //! \return bool
    bool operator==(const vector<type, value> &) const;
  };

#ifdef EIGEN
  using Vector = class vector<Eigen::VectorXd, double>;
  using SparseVector = class vector<Eigen::SparseVector<double>, double>;
#endif

#ifdef BOOST
  using Vector = class vector<boost::numeric::ublas::vector<double>, double>;
#endif

}


namespace std
{
  template <typename T, typename V>
  struct hash<sdm::vector<T, V>>
  {
    typedef sdm::vector<T, V> argument_type;
    typedef std::size_t result_type;
    result_type operator()(argument_type const &in) const
    {
      size_t size = in.size();
      size_t seed = 0;
      for (size_t i = 0; i < size; i++)
      {
        //Combine the hash of the current vector with the hashes of the previous ones
        sdm::hash_combine(seed, in[i]);
      }
      return seed;
    }
  };
}

