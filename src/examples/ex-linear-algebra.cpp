#include <iostream>
#include <sdm/types.hpp>
#include <sdm/utils/linear_algebra/sdms_vector.hpp>
#include <sdm/utils/linear_algebra/matrix.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>

using namespace sdm;

int main(int, char **)
{
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
