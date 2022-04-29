#define BOOST_TEST_MODULE hsviTest

#include <boost/test/unit_test.hpp>
#include <sdm/common.hpp>
#include <sdm/algorithms.hpp>
#include <sdm/algorithms/planning/hsvi.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/utils/value_function/initializer.hpp>

using namespace sdm;

std::shared_ptr<HSVI> make(std::string filename, int horizon, int memory = 1)
{
    return std::static_pointer_cast<HSVI>(sdm::algo::make("o-HSVI", filename, horizon, 1., 0.000001, 1000, 10000, "", memory));
}

BOOST_AUTO_TEST_CASE(Solver_oMDP)
{
    auto algorithm = make("mabc.dpomdp", 10, 1);
    algorithm->initialize();
    algorithm->solve();
    double resultat = algorithm->getLowerBound()->getValueAt(algorithm->getWorld()->getInitialState());
    BOOST_CHECK_EQUAL(round(resultat * 100), 929);

    algorithm = make("recycling.dpomdp", 5, 1);
    algorithm->getUpperBound()->setInitializer(sdm::initializer::registry::make("Pomdp", algorithm->getWorld()));
    algorithm->initialize();
    algorithm->solve();
    resultat = algorithm->getLowerBound()->getValueAt(algorithm->getWorld()->getInitialState());
    BOOST_CHECK_EQUAL(round(resultat * 1000), 16486);

    algorithm = make("tiger.dpomdp", 3, 3);
    algorithm->getUpperBound()->setInitializer(sdm::initializer::registry::make("Pomdp", algorithm->getWorld()));
    algorithm->initialize();
    algorithm->solve();
    resultat = algorithm->getLowerBound()->getValueAt(algorithm->getWorld()->getInitialState());
    BOOST_CHECK_EQUAL(round(resultat * 10000), 51908);
}