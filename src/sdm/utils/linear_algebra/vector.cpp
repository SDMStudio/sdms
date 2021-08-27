/*=============================================================================
Copyright (C) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#include <algorithm>

#include <sdm/utils/linear_algebra/vector.hpp>

namespace sdm{
  template<typename type, typename value>
  vector<type, value>::vector(){
    this->_size_ = 0;
  }

  template<typename type, typename value>
  vector<type, value>::vector(const vector& arg){
    this->_size_ = arg._size_;
    this->container = arg.container;
  }

  template<typename type, typename value>
  vector<type, value>::vector(std::size_t _size_){
    this->resize(_size_);
  }

  template<typename type, typename value>
  vector<type, value>::~vector(){}

  template<typename type, typename value>
  const type& vector<type, value>::getContainer() const {
    return this->container;
  }

  template<typename type, typename value>
  void vector<type, value>::setContainer(const type& container){
    this->container = container;
  }

  template<typename type, typename value>
  void vector<type, value>::resize(std::size_t _size_){
    this->_size_ = _size_;
    this->container = type(_size_);
  }

  template<typename type, typename value>
  vector<type,value> vector<type,value>::transpose() const{
    vector<type,value> vnew;
    vnew.container =  this->container.transpose();
    return vnew;
  }


  template<typename type, typename value>
  void vector<type, value>::init(value arg){
    size_t i;
    for(i=0; i<_size_; ++i){
      (*this)[i] = arg;   //<! TODO should avoid explicit enumeration of all states.(intial states)
    }
  }

  template<typename type, typename value>
  size_t vector<type, value>::size() const{
    return this->container.size();
  }

  template<typename type, typename value>
  value vector<type, value>::operator^(const vector<type, value>& arg) const {
    return this->container.dot(arg.container);
  }

  template<typename type, typename value>
  bool vector<type, value>::operator==(const vector<type, value>& arg) const {
    return this->container == arg.container;
  }

  /**
  Array subscript operator '[]'
  */
  template<typename type, typename value>
  value& vector<type, value>::operator[](std::size_t idx){
    return this->container(idx);
  }

  /**
  Min of entries -- using methods from parent classes.
  */
  template<typename type, typename value>
  value vector<type, value>::min() const{
    return -99999999;
  }


  /**
  Sum of entries -- using methods from parent classes.
  */
  template<typename type, typename value>
  value vector<type, value>::sum(){
    return this->container.sum();
  }

  /**
  Max of entries -- using methods from parent classes.
  */
  template<typename type, typename value>
  value vector<type, value>::max() const{
    return 9999999;
  }

  template<typename type, typename value>
  value vector<type, value>::norm_2() const{
    value v = 0.0;
    for(size_t i=0; i<_size_; ++i){
      v += std::pow((*this)[i], 2);   //<! TODO should avoid explicit enumeration of all states.
    }

    return v;
  }

  template<typename type, typename value>
  value vector<type, value>::norm_1() const{
    value v = 0.0;
    for(size_t i=0; i<_size_; ++i){
      v += std::abs((*this)[i]);   //<! TODO should avoid explicit enumeration of all states.
    }

    return v;
  }

  template<typename type, typename value>
  value vector<type, value>::norm_sawtooth(const vector& arg) const{
    double min = 1.0;

    //<! computes the distance metric, be carefull some histories may not be part of 's'
    for(size_t i=0; i<_size_; ++i) if(this->container[i] > 0){
      min = std::min(min, arg.container[i] / this->container[i]);
    }

    return min;
  }

  template<typename type, typename value>
  value vector<type, value>::operator[](std::size_t idx) const {
    return this->container(idx);
  }


  #ifdef EIGEN
    #ifdef DENSE
      template<>
      double vector<Eigen::VectorXd, double>::norm_1() const{
        return this->container.lpNorm<1>();
      }

      template<>
      double vector<Eigen::VectorXd, double>::norm_2() const{
        return this->container.lpNorm<2>();
      }

      template<>
      double vector<Eigen::VectorXd, double>::max() const{
        return this->container.maxCoeff();
      }

      template<>
      double vector<Eigen::VectorXd, double>::min() const{
        return this->container.minCoeff();
      }

      template class vector<Eigen::VectorXd, double>;
    #endif
    #ifdef SPARSE
      template<>
      double vector<Eigen::VectorXd, double>::norm_1() const{
        return this->container.lpNorm<1>();
      }

      template<>
      double vector<Eigen::VectorXd, double>::norm_2() const{
        return this->container.lpNorm<2>();
      }

      template<>
      double& vector<Eigen::SparseVector<double>, double>::operator[](std::size_t idx){
        return this->container.coeffRef(idx);
      }

      template<>
      double vector<Eigen::SparseVector<double>, double>::operator[](std::size_t idx) const{
        return this->container.coeff(idx);
      }

      template<>
      bool vector<Eigen::SparseVector<double>, double>::operator==(const vector<Eigen::SparseVector<double>, double>& arg) const {
        std::size_t size1 = this->container.size();
        std::size_t size2 = arg.container.size();
        if( size1 != size2 ) return false;

        return arg.container.isApprox(this->container, 0.0001);
      }

      template class vector<Eigen::SparseVector<double>, double>;
    #endif
  #endif

  #ifdef BOOST
    #ifdef DENSE

      template<>
      double vector<boost::numeric::ublas::vector<double>, double>::norm_1() const{
        return boost::numeric::ublas::norm_1(this->container);
      }

      template<>
      double vector<boost::numeric::ublas::vector<double>, double>::norm_2() const{
        return boost::numeric::ublas::norm_2(this->container);
      }

      template<>
      vector<boost::numeric::ublas::vector<double>, double> vector<boost::numeric::ublas::vector<double>, double>::transpose() const{
        vector<boost::numeric::ublas::vector<double>, double> vnew;
        vnew.container =  boost::numeric::ublas::trans(this->container);
        return vnew;
      }

      template<>
      double vector<boost::numeric::ublas::vector<double>, double>::operator[](std::size_t idx) const {
        return this->container[idx];
      }

      template<>
      double vector<boost::numeric::ublas::vector<double>, double>::min() const{
        return *std::min_element(this->container.begin(), this->container.end());
      }

      template<>
      double vector<boost::numeric::ublas::vector<double>, double>::max() const{
        return *std::max_element(this->container.begin(), this->container.end());
      }

      template<>
      double vector<boost::numeric::ublas::vector<double>, double>::sum(){
        return boost::numeric::ublas::sum(this->container);
      }

      /**
      Scalar product - using a binary operator '^'
      */
      template<>
      double vector<boost::numeric::ublas::vector<double>, double>::operator^(const vector<boost::numeric::ublas::vector<double>, double>& arg) const {
        size_t i, _size_ = arg.container.size();

        double result = 0.0;
        for(i=0; i<_size_; i++){
          result += this->container[i] * arg.container[i];
        }

        return result;
      }

      /**
      Equality -- using binary operator '=='
      */
      template<>
      bool vector<boost::numeric::ublas::vector<double>, double>::operator==(const vector<boost::numeric::ublas::vector<double>, double>& arg) const {
        std::size_t i, size1 = this->container.size();
        std::size_t size2 = arg.container.size();

        if( size1 != size2 ) return false;
        for(i=0; i<size1; ++i) if(this->container[i] != arg.container[i]) return false;

        return true;
      }

      template class vector<boost::numeric::ublas::vector<double>, double>;
    #endif
    #ifdef SPARSE
      template class vector<boost::numeric::ublas::vector<double>, double>;
    #endif
  #endif



}
