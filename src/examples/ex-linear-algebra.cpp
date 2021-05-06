#include <iostream>
#include <sdm/types.hpp>
#include <sdm/utils/linear_algebra/sdms_vector.hpp>
#include <sdm/utils/linear_algebra/sdms_matrix.hpp>
#include <sdm/utils/linear_algebra/matrix.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>

using namespace sdm;

int main(int, char **)
{

    DenseVector<number, double> test_vector(3), test_vector2(3);
    std::cout << test_vector << std::endl;
    test_vector[0] = 5;
    test_vector[2] = 5;
    test_vector2[1] = 1;
    test_vector2[2] = 2;
    std::cout << test_vector << std::endl;
    std::cout << test_vector2 << std::endl;
    std::cout << (test_vector ^ test_vector2) << std::endl;
    std::cout << boost::numeric::ublas::inner_prod(test_vector, test_vector2) << std::endl;

    DenseMatrix<number, number, double> test_matrix(4, 3);
    std::cout << test_matrix << std::endl;

    std::cout << (boost::numeric::ublas::prod(test_matrix, test_vector)) << std::endl;

    Vector dense_vector(3);
    dense_vector[0] = 10;
    dense_vector[1] = 1;
    std::cout << "Dense Vector = " << dense_vector << std::endl;

    Matrix dense_matrix(4, 3);
    dense_matrix(0, 0) = 168;
    dense_matrix(0, 1) = 10;
    std::cout << "Dense Matrix = " << dense_matrix << std::endl;

    std::cout << "Multiplication = " << (dense_matrix * dense_vector) << std::endl;

    SparseVector<number, double> sparse_vector(3);
    std::cout << "Sparse Vector = " << sparse_vector << std::endl;

    return 0;
}
