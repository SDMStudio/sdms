#include <iostream>
#include <sdm/types.hpp>
#include <sdm/world/discrete_pomdp.hpp>
#include <sdm/world/discrete_decpomdp.hpp>
#include <sdm/utils/linear_algebra/sdms_vector.hpp>
#include <sdm/utils/linear_algebra/sdms_matrix.hpp>
#include <sdm/utils/linear_algebra/matrix.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/core/state/beliefs.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>

using namespace sdm;

int main(int argc, char **argv)
{

    std::string filename;

    if (argc > 1)
    {
        filename = argv[1];
    }
    else
    {
        std::cerr << "Error: Require 1 input file." << std::endl;
        return 1;
    }

    using TState = number;
    using TAction = number;
    using TObservation = number;

    using TBelief = BeliefState<TState>;
    // using TBeliefStruct = BeliefStateGraph<TBelief, TAction, TObservation>;

    using TWorld = BeliefMDP<TBelief, TAction, TObservation>;

    auto belief_mdp = std::make_shared<TWorld>(filename);

    // auto belief = std::make_shared<TBeliefStruct>(belief_mdp->getStateSpace()->getAll(), belief_mdp->getStartDistrib().probabilities());
    auto belief = std::make_shared<TBeliefStruct>(belief_mdp->getStateSpace()->getAll(), belief_mdp->getStartDistrib().probabilities(), belief_mdp->getObsDynamics()->getDynamics());

    std::cout << *belief << std::endl;
    DenseMatrix<number, number> dyn(2, 2);
    dyn(0,1) = 2;

    std::cout << (dyn ^ *belief) << std::endl;
    // std::cout << *belief_mdp << std::endl;

    std::cout << "PRECISION BeliefState = " << BeliefState<number>::PRECISION << std::endl;
    std::cout << "PRECISION DenseVector = " << DenseVector<number>::PRECISION << std::endl;
    std::cout << "PRECISION SparseVector = " << SparseVector<number>::PRECISION << std::endl;

    BaseBeliefState<std::string, DenseVector> b(3, 0.), b2(3, 0.), b3(3, 2.), b4;

    // std::cout << b.iterator_.size() << std::endl;

    b.setIndexes({"State 0", "State 1", "State 2"});
    b2.setIndexes({"State 0", "State 1", "State 2"});
    b3.setIndexes({"State 0", "State 1", "State 2"});
    b.setProbabilityAt("State 0", 0.6);
    b.setProbabilityAt("State 1", 0.4);
    // std::cout << b.iterator_.size() << std::endl;

    b2.setProbabilityAt("State 0", 0.601);
    b2.setProbabilityAt("State 1", 0.4000001);

    b3.setProbabilityAt("State 0", 0.6);
    b3.setProbabilityAt("State 1", 0.4);

    std::cout << b << std::endl;

    std::cout << (b ^ b3) << std::endl;

    std::cout << (b == b2) << std::endl;
    std::cout << (b == b3) << std::endl;

    DenseVector<number> test_vector(3), test_vector2(3);
    SparseVector<number> test_sparse_vector(3), test_sparse_vector2({1, 0, 3});

    test_sparse_vector[0] = 1;
    test_sparse_vector[2] = 3;

    std::cout << (test_sparse_vector2 == test_sparse_vector) << std::endl;
    std::cout << (test_sparse_vector == test_sparse_vector2) << std::endl;

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
