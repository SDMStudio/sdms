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
    DenseVector<number> test_vector(3), test_vector2(3);
    SparseVector<number> test_sparse_vector(3), test_sparse_vector2({3, 4, 5});

    test_sparse_vector[0] = 1;
    test_sparse_vector[2] = 3;

    // std::cout << "test_sparse_vector2 ^ v=" << (test_sparse_vector2 ^ v) << std::endl;
    // std::cout << "v ^ test_sparse_vector2=" << (v ^ test_sparse_vector2) << std::endl;

    std::cout << "test_sparse_vector=" << (test_sparse_vector <= test_sparse_vector2) << std::endl;
    std::cout << "test_sparse_vector=" << (test_sparse_vector2 <= test_sparse_vector) << std::endl;
    std::cout << "test_sparse_vector2=" << test_sparse_vector2 << std::endl;

    std::cout << "test_sparse_vector ^ test_sparse_vector2=" << (test_sparse_vector ^ test_sparse_vector2) << std::endl;
    test_vector[0] = 5;
    test_vector[2] = 5;

    test_vector2[1] = 1;
    test_vector2[2] = 2;
    std::cout << "test_vector=" << test_vector << std::endl;
    std::cout << "test_vector.transpose()=" << test_vector.transpose() << std::endl;
    std::cout << "test_vector2=" << test_vector2 << std::endl;
    std::cout << "test_vector ^ test_sparse_vector=" << (test_vector ^ test_sparse_vector) << std::endl;

    SparseMatrix<number, number> test_matrix(3, 3);
    test_matrix(1, 0) = 4;
    std::cout << "test_matrix=" << test_matrix << std::endl;
    std::cout << "test_matrix.transpose()=" << test_matrix.transpose() << std::endl;

    test_vector = (test_matrix ^ test_vector);
    test_vector = (test_matrix ^ test_sparse_vector);
    std::cout << "test_matrix ^ test_vector " << (test_matrix ^ test_vector) << std::endl;
    std::cout << "test_matrix ^ test_sparse_vector " << (test_matrix ^ test_sparse_vector) << std::endl;

    std::cout << "test_sparse_vector.norm_1()=" << test_sparse_vector.norm_1() << std::endl;
    std::cout << "test_sparse_vector.norm_2()=" << test_sparse_vector.norm_2() << std::endl;
    std::cout << "test_sparse_vector.sum()=" << test_sparse_vector.sum() << std::endl;
    std::cout << "test_sparse_vector.max()=" << test_sparse_vector.max() << std::endl;
    std::cout << "test_sparse_vector.argmax()=" << test_sparse_vector.argmax() << std::endl;
    std::cout << "test_sparse_vector.min()=" << test_sparse_vector.min() << std::endl;
    std::cout << "test_sparse_vector.argmin()=" << test_sparse_vector.argmin() << std::endl;

    return 0;
}
