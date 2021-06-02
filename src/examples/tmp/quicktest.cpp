#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/linear_algebra/sdms_vector.hpp>
#include <sdm/core/state/discrete_state.hpp>


using namespace sdm;

int main(int argc, char **argv)
{

    std::cout << "PRECISION DenseVector = " << DenseVector<number>::PRECISION << std::endl;
    std::cout << "PRECISION SparseVector = " << SparseVector<number>::PRECISION << std::endl;

    // Creation of Dense Vector 1

    std::unordered_map<std::shared_ptr<DiscreteState>,size_t> map_element_to_index;
    std::unordered_map<std::shared_ptr<DiscreteState>,double> map_element_to_value;

    auto element_state_0 = std::make_shared<DiscreteState>(0); // Create State 0
    auto element_state_5 = std::make_shared<DiscreteState>(5); // Create State 5

    map_element_to_index.emplace(element_state_0,1); // The State 0 is associated with the index 5
    map_element_to_value.emplace(element_state_0,20); // The State 0 has a value of 20

    map_element_to_index.emplace(element_state_5,0); // The State 5 is associated with the index 3
    map_element_to_value.emplace(element_state_5,5); // The State 5 has a value of 5

    auto shared_map_element_to_index = std::make_shared<std::unordered_map<std::shared_ptr<DiscreteState>,size_t>>(map_element_to_index);
    auto shared_map_element_to_value = std::make_shared<std::unordered_map<std::shared_ptr<DiscreteState>,double>>(map_element_to_value);

    DenseVector<std::shared_ptr<DiscreteState>> test_vector(shared_map_element_to_index,shared_map_element_to_value);

    // Creation of Dense Vector 2

    std::unordered_map<std::shared_ptr<DiscreteState>,double> map_element_to_value_2;

    map_element_to_value_2.emplace(element_state_0,50); // The State 0 has a value of 20

    map_element_to_value_2.emplace(element_state_5,5); // The State 5 has a value of 5

    auto shared_map_element_to_value_2 = std::make_shared<std::unordered_map<std::shared_ptr<DiscreteState>,double>>(map_element_to_value_2);

    DenseVector<std::shared_ptr<DiscreteState>> test_vector_2(shared_map_element_to_index,shared_map_element_to_value_2);

    // Basic Function of SDMS Vector

    std::cout<<"Print Test Vector"<<test_vector.str()<<std::endl;

    std::cout<<"Test Sum : "<<test_vector.sum()<<std::endl;
    std::cout<<"Value State 0 : "<<test_vector.at(element_state_0)<<std::endl;

    std::cout<<"Return all Indexes" <<test_vector.getIndexes()<<std::endl;

    std::cout<<"Get Norm 1 :"<<test_vector.norm_1()<<std::endl;
    std::cout<<"Get Norm 2 :"<<test_vector.norm_2()<<std::endl;

    std::cout<<"Get Min : "<<test_vector.min()<<std::endl;
    std::cout<<"Get Max : "<<test_vector.max()<<std::endl;

    std::cout <<"Test Vector Equivalence : "<< (test_vector == test_vector_2) << std::endl;
    std::cout <<"Test Vector <= : "<< (test_vector <= test_vector_2) << std::endl;

    std::cout<<"Modification of value State 0 from 20 to 50 "<<std::endl;
    test_vector.setValueAt(element_state_0,50);
    std::cout<<"New Value At State 0 : "<<test_vector.at(element_state_0)<<std::endl;

    std::cout <<"Test Vector Equivalence : "<< (test_vector == test_vector_2) << std::endl;
    std::cout<<"Dot Bewtween the vector "<<test_vector.dot(test_vector_2)<<std::endl;

    // using TState = number;
    // using TAction = number;
    // using TObservation = number;

    // using TBelief = BeliefState<TState>;
    // // using TBeliefStruct = BeliefStateGraph<TBelief, TAction, TObservation>;

    // using TWorld = BeliefMDP<TBelief, TAction, TObservation>;

    // auto belief_mdp = std::make_shared<TWorld>(filename);

    // // auto belief = std::make_shared<TBeliefStruct>(belief_mdp->getStateSpace()->getAll(), belief_mdp->getStartDistrib().probabilities());
    // auto belief = std::make_shared<TBeliefStruct>(belief_mdp->getStateSpace()->getAll(), belief_mdp->getStartDistrib().probabilities(), belief_mdp->getObsDynamics()->getDynamics());

    // std::cout << *belief << std::endl;
    // DenseMatrix<number, number> dyn(2, 2);
    // dyn(0,1) = 2;

    // std::cout << (dyn ^ *belief) << std::endl;
    // // std::cout << *belief_mdp << std::endl;

    // std::cout << "PRECISION BeliefState = " << BeliefState<number>::PRECISION << std::endl;
    // std::cout << "PRECISION DenseVector = " << DenseVector<number>::PRECISION << std::endl;
    // std::cout << "PRECISION SparseVector = " << SparseVector<number>::PRECISION << std::endl;

    // BaseBeliefState<std::string, DenseVector> b(3, 0.), b2(3, 0.), b3(3, 2.), b4;

    // // std::cout << b.iterator_.size() << std::endl;

    // b.setIndexes({"State 0", "State 1", "State 2"});
    // b2.setIndexes({"State 0", "State 1", "State 2"});
    // b3.setIndexes({"State 0", "State 1", "State 2"});
    // b.setProbabilityAt("State 0", 0.6);
    // b.setProbabilityAt("State 1", 0.4);
    // // std::cout << b.iterator_.size() << std::endl;

    // b2.setProbabilityAt("State 0", 0.601);
    // b2.setProbabilityAt("State 1", 0.4000001);

    // b3.setProbabilityAt("State 0", 0.6);
    // b3.setProbabilityAt("State 1", 0.4);

    // std::cout << b << std::endl;

    // std::cout << (b ^ b3) << std::endl;

    // std::cout << (b == b2) << std::endl;
    // std::cout << (b == b3) << std::endl;

    // DenseVector<number> test_vector(3), test_vector2(3);
    // SparseVector<number> test_sparse_vector(3), test_sparse_vector2({1, 0, 3});

    // test_sparse_vector[0] = 1;
    // test_sparse_vector[2] = 3;

    // std::cout << (test_sparse_vector2 == test_sparse_vector) << std::endl;
    // std::cout << (test_sparse_vector == test_sparse_vector2) << std::endl;

    // // std::cout << "test_sparse_vector2 ^ v=" << (test_sparse_vector2 ^ v) << std::endl;
    // // std::cout << "v ^ test_sparse_vector2=" << (v ^ test_sparse_vector2) << std::endl;

    // std::cout << "test_sparse_vector=" << (test_sparse_vector <= test_sparse_vector2) << std::endl;
    // std::cout << "test_sparse_vector=" << (test_sparse_vector2 <= test_sparse_vector) << std::endl;
    // std::cout << "test_sparse_vector2=" << test_sparse_vector2 << std::endl;

    // std::cout << "test_sparse_vector ^ test_sparse_vector2=" << (test_sparse_vector ^ test_sparse_vector2) << std::endl;
    // test_vector[0] = 5;
    // test_vector[2] = 5;

    // test_vector2[1] = 1;
    // test_vector2[2] = 2;
    // std::cout << "test_vector=" << test_vector << std::endl;
    // std::cout << "test_vector.transpose()=" << test_vector.transpose() << std::endl;
    // std::cout << "test_vector2=" << test_vector2 << std::endl;
    // std::cout << "test_vector ^ test_sparse_vector=" << (test_vector ^ test_sparse_vector) << std::endl;

    // SparseMatrix<number, number> test_matrix(3, 3);
    // test_matrix(1, 0) = 4;
    // std::cout << "test_matrix=" << test_matrix << std::endl;
    // std::cout << "test_matrix.transpose()=" << test_matrix.transpose() << std::endl;

    // test_vector = (test_matrix ^ test_vector);
    // test_vector = (test_matrix ^ test_sparse_vector);
    // std::cout << "test_matrix ^ test_vector " << (test_matrix ^ test_vector) << std::endl;
    // std::cout << "test_matrix ^ test_sparse_vector " << (test_matrix ^ test_sparse_vector) << std::endl;

    // std::cout << "test_sparse_vector.norm_1()=" << test_sparse_vector.norm_1() << std::endl;
    // std::cout << "test_sparse_vector.norm_2()=" << test_sparse_vector.norm_2() << std::endl;
    // std::cout << "test_sparse_vector.sum()=" << test_sparse_vector.sum() << std::endl;
    // std::cout << "test_sparse_vector.max()=" << test_sparse_vector.max() << std::endl;
    // std::cout << "test_sparse_vector.argmax()=" << test_sparse_vector.argmax() << std::endl;
    // std::cout << "test_sparse_vector.min()=" << test_sparse_vector.min() << std::endl;
    // std::cout << "test_sparse_vector.argmin()=" << test_sparse_vector.argmin() << std::endl;

    return 0;
}
