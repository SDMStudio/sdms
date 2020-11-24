/*=============================================================================
Copyright (C) 2016 Jilles Steeve Dibangoye
==============================================================================*/

#include <sdm/utils/linear_algebra/matrix.hpp>

namespace sdm
{
  template <typename mtype, typename vtype, typename value>
  vector<vtype, value> matrix<mtype, vtype, value>::row(number i)
  {
    vector<vtype, value> r;
    r.setContainer(this->container.row(i));
    return r;
  }

  template <typename mtype, typename vtype, typename value>
  void matrix<mtype, vtype, value>::compress() {}

  template <typename mtype, typename vtype, typename value>
  matrix<mtype, vtype, value> operator*(const matrix<mtype, vtype, value> &arg1, const matrix<mtype, vtype, value> &arg2)
  {
    mtype container = arg1.container * arg2.container;
    matrix<mtype, vtype, value> m(container.rows(), container.cols());
    m.container = container;
    return m;
  }

  template <typename mtype, typename vtype, typename value>
  bool operator==(const matrix<mtype, vtype, value> &arg1, const matrix<mtype, vtype, value> &arg2)
  {
    return arg1.container == arg2.container;
  }

  template <typename mtype, typename vtype, typename value>
  matrix<mtype, vtype, value> operator^(const vector<vtype, value> &v, const matrix<mtype, vtype, value> &m)
  {
    std::size_t i, j, size1 = m.rows(), size2 = m.cols();
    matrix<mtype, vtype, value> result(size1, size2);

    for (i = 0; i < size1; ++i)
      for (j = 0; j < size2; ++j)
        result(j, i) = v[j] * m(i, j);

    return result;
  }

  template <typename mtype, typename vtype, typename value>
  matrix<mtype, vtype, value>::matrix() {}

  template <typename mtype, typename vtype, typename value>
  matrix<mtype, vtype, value>::matrix(std::size_t rows, std::size_t cols)
  {
    this->resize(rows, cols);
  }

  template <typename mtype, typename vtype, typename value>
  matrix<mtype, vtype, value>::~matrix() {}

  //TODO please find a proper way to do this!
  template <typename mtype, typename vtype, typename value>
  void matrix<mtype, vtype, value>::resize(std::size_t rows, std::size_t cols)
  {
    this->container.resize(rows, cols);
  }

  /**
  Specialization of operator '()'
  */
  template <typename mtype, typename vtype, typename value>
  value &matrix<mtype, vtype, value>::operator()(std::size_t row, std::size_t col)
  {
    return this->container(row, col);
  }

  template <typename mtype, typename vtype, typename value>
  value matrix<mtype, vtype, value>::operator()(std::size_t row, std::size_t col) const
  {
    return this->container(row, col);
  }

  template <typename mtype, typename vtype, typename value>
  matrix<mtype, vtype, value> matrix<mtype, vtype, value>::transpose() const
  {
    matrix<mtype, vtype, value> m;

#ifdef EIGEN
    m.container = this->container.transpose();
#endif
#ifdef BOOST
    m.container = boost::numeric::ublas::trans(this->container);
#endif

    return m;
  }

  /**
  Matrix / Vector product
  */
  template <typename mtype, typename vtype, typename value>
  vector<vtype, value> operator*(const matrix<mtype, vtype, value> &arg1, const vector<vtype, value> &arg2)
  {
    vtype container = arg1.container * arg2.getContainer();
    vector<vtype, value> v(container.size());
    v.setContainer(container);
    return v;
  }

  /**
  Get Matrix number of rows
  */
  template <typename mtype, typename vtype, typename value>
  std::size_t matrix<mtype, vtype, value>::rows()
  {
    return this->container.rows();
  }

  /**
  Get Matrix number of columns
  */
  template <typename mtype, typename vtype, typename value>
  std::size_t matrix<mtype, vtype, value>::cols()
  {
    return this->container.cols();
  }

#ifdef EIGEN
#ifdef DENSE
  template class matrix<Eigen::MatrixXd, Eigen::VectorXd, double>;
  template Vector operator*<Eigen::MatrixXd, Eigen::VectorXd, double>(Matrix const &, Vector const &);
#endif
#ifdef SPARSE
  template <>
  void matrix<Eigen::SparseMatrix<double, Eigen::RowMajor>, Eigen::SparseVector<double>, double>::compress()
  {
    this->container.makeCompressed();
  }

  template <>
  double &matrix<Eigen::SparseMatrix<double, Eigen::RowMajor>, Eigen::SparseVector<double>, double>::operator()(std::size_t row, std::size_t col)
  {
    return this->container.coeffRef(row, col);
  }

  template <>
  double matrix<Eigen::SparseMatrix<double, Eigen::RowMajor>, Eigen::SparseVector<double>, double>::operator()(std::size_t row, std::size_t col) const
  {
    return this->container.coeff(row, col);
  }

  template class matrix<Eigen::SparseMatrix<double, Eigen::RowMajor>, Eigen::SparseVector<double>, double>;
  template Vector operator*<Eigen::SparseMatrix<double, Eigen::RowMajor>, Eigen::SparseVector<double>, double>(Matrix const &, Vector const &);
#endif
#endif

#ifdef BOOST
#ifdef DENSE
  template <>
  std::size_t matrix<boost::numeric::ublas::matrix<double>, boost::numeric::ublas::vector<double>, double>::cols()
  {
    return this->container.size2();
  }

  template <>
  std::size_t matrix<boost::numeric::ublas::matrix<double>, boost::numeric::ublas::vector<double>, double>::rows()
  {
    return this->container.size1();
  }

  template <>
  bool operator==(const matrix<boost::numeric::ublas::mapped_matrix<double>, boost::numeric::ublas::vector<double>, double> &arg1, const matrix<boost::numeric::ublas::mapped_matrix<double>, boost::numeric::ublas::vector<double>, double> &arg2)
  {
    std::size_t i, j, size1 = arg1.container.size1(), size2 = arg2.container.size2();

    for (i = 0; i < size1; i++)
      for (j = 0; j < size2; ++j)
        if (arg1.container(i, j) != arg2.container(i, j))
          return false;

    return true;
  }

  template <>
  vector<boost::numeric::ublas::vector<double>, double> operator*(const matrix<boost::numeric::ublas::matrix<double>, boost::numeric::ublas::vector<double>, double> &arg1, const vector<boost::numeric::ublas::vector<double>, double> &arg2)
  {
    boost::numeric::ublas::vector<double> container = boost::numeric::ublas::prod(arg1.container, arg2.getContainer());
    vector<boost::numeric::ublas::vector<double>, double> v(container.size());
    v.setContainer(container);
    return v;
  }

  template <>
  vector<boost::numeric::ublas::vector<double>, double> operator*(const matrix<boost::numeric::ublas::mapped_matrix<double>, boost::numeric::ublas::vector<double>, double> &arg1, const vector<boost::numeric::ublas::vector<double>, double> &arg2)
  {
    boost::numeric::ublas::vector<double> container = boost::numeric::ublas::prod(arg1.container, arg2.getContainer());
    vector<boost::numeric::ublas::vector<double>, double> v(container.size());
    v.setContainer(container);
    return v;
  }

  /**
      Matrix product
      */
  template <>
  matrix<boost::numeric::ublas::matrix<double>, boost::numeric::ublas::vector<double>, double> operator*(const matrix<boost::numeric::ublas::matrix<double>, boost::numeric::ublas::vector<double>, double> &arg1, const matrix<boost::numeric::ublas::matrix<double>, boost::numeric::ublas::vector<double>, double> &arg2)
  {
    boost::numeric::ublas::matrix<double> container = boost::numeric::ublas::prod(arg1.container, arg2.container);
    matrix<boost::numeric::ublas::matrix<double>, boost::numeric::ublas::vector<double>, double> m(container.size1(), container.size2());
    m.container = container;
    return m;
  }

  /**
      Matrix egality
      */
  template <>
  bool operator==(const matrix<boost::numeric::ublas::matrix<double>, boost::numeric::ublas::vector<double>, double> &arg1, const matrix<boost::numeric::ublas::matrix<double>, boost::numeric::ublas::vector<double>, double> &arg2)
  {
    std::size_t i, j, size1 = arg1.container.size1(), size2 = arg2.container.size2();
    for (i = 0; i < size1; i++)
      for (j = 0; j < size2; ++j)
        if (arg1.container(i, j) != arg2.container(i, j))
          return false;

    return true;
  }

  template <>
  vector<boost::numeric::ublas::vector<double>, double> matrix<boost::numeric::ublas::matrix<double>, boost::numeric::ublas::vector<double>, double>::row(number i)
  {
    vector<boost::numeric::ublas::vector<double>, double> r;
    r.setContainer(boost::numeric::ublas::row(this->container, i));
    return r;
  }

  template <>
  vector<boost::numeric::ublas::vector<double>, double> matrix<boost::numeric::ublas::mapped_matrix<double>, boost::numeric::ublas::vector<double>, double>::row(number i)
  {
    vector<boost::numeric::ublas::vector<double>, double> r;
    r.setContainer(boost::numeric::ublas::row(this->container, i));
    return r;
  }

  template class matrix<boost::numeric::ublas::matrix<double>, boost::numeric::ublas::vector<double>, double>;
#endif
#ifdef SPARSE
  template <>
  std::size_t matrix<boost::numeric::ublas::mapped_matrix<double>, boost::numeric::ublas::vector<double>, double>::cols()
  {
    return this->container.size2();
  }

  template <>
  matrix<boost::numeric::ublas::mapped_matrix<double>, boost::numeric::ublas::vector<double>, double>::matrix(std::size_t rows, std::size_t cols)
  {
    this->container = boost::numeric::ublas::mapped_matrix<double>(rows, cols, rows * cols);
  }

  template <>
  double &matrix<boost::numeric::ublas::mapped_matrix<double>, boost::numeric::ublas::vector<double>, double>::operator()(std::size_t row, std::size_t col)
  {
    return this->container.insert_element(row, col, row * col);
  }

  template <>
  double matrix<boost::numeric::ublas::mapped_matrix<double>, boost::numeric::ublas::vector<double>, double>::operator()(std::size_t row, std::size_t col) const
  {
    return this->container(row, col);
  }

  template <>
  matrix<boost::numeric::ublas::mapped_matrix<double>, boost::numeric::ublas::vector<double>, double> operator*(const matrix<boost::numeric::ublas::mapped_matrix<double>, boost::numeric::ublas::vector<double>, double> &arg1, const matrix<boost::numeric::ublas::mapped_matrix<double>, boost::numeric::ublas::vector<double>, double> &arg2)
  {
    boost::numeric::ublas::mapped_matrix<double> container = boost::numeric::ublas::prod(arg1.container, arg2.container);
    matrix<boost::numeric::ublas::mapped_matrix<double>, boost::numeric::ublas::vector<double>, double> m(container.size1(), container.size2());
    m.container = container;
    return m;
  }

  /**
      Matrix / Vector point to point product
      */
  template <>
  matrix<boost::numeric::ublas::mapped_matrix<double>, boost::numeric::ublas::vector<double>, double> operator^(const vector<boost::numeric::ublas::vector<double>, double> &v, const matrix<boost::numeric::ublas::mapped_matrix<double>, boost::numeric::ublas::vector<double>, double> &m)
  {
    std::size_t i, j, size1 = m.container.size1(), size2 = m.container.size2();
    matrix<boost::numeric::ublas::mapped_matrix<double>, boost::numeric::ublas::vector<double>, double> result(size1, size2);
    for (i = 0; i < size1; ++i)
      for (j = 0; j < size2; ++j)
        result(j, i) = v[j] * m(i, j);
    return result;
  }

  template <>
  std::size_t matrix<boost::numeric::ublas::mapped_matrix<double>, boost::numeric::ublas::vector<double>, double>::rows()
  {
    return this->container.size1();
  }

  template class matrix<boost::numeric::ublas::mapped_matrix<double>, boost::numeric::ublas::vector<double>, double>;
#endif
#endif

} // namespace sdm
