#define BOOST_TEST_MODULE extbHsviTest

#include <boost/test/unit_test.hpp>
#include <sdm/common.hpp>
#include <sdm/algorithms.hpp>
#include <sdm/algorithms/planning/hsvi.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/utils/value_function/initializer.hpp>

using namespace sdm;

std::shared_ptr<HSVI> make(std::string filename, std::string formalism, int horizon, int memory = 1)
{
    return std::static_pointer_cast<HSVI>(sdm::algo::make("HSVI", filename, formalism, horizon, 1., 0.000001, 1000, 10000, "", memory));
}

BOOST_AUTO_TEST_CASE(Solver_ext_bMDP)
{
    auto algorithm = make("mabc.dpomdp", "ext-bMDP", 10);
    algorithm->initialize();
    algorithm->solve();
    double resultat = algorithm->getLowerBound()->getValueAt(algorithm->getWorld()->getInitialState());
    BOOST_CHECK_EQUAL(round(resultat * 100), 929);

    algorithm = make("recycling.dpomdp", "ext-bMDP", 5);
    algorithm->getUpperBound()->setInitializer(sdm::initializer::registry::make("Mdp", algorithm->getWorld()));
    algorithm->initialize();
    algorithm->solve();
    resultat = algorithm->getLowerBound()->getValueAt(algorithm->getWorld()->getInitialState());
    BOOST_CHECK_EQUAL(round(resultat * 10000), 175309);

    algorithm = make("tiger.dpomdp", "ext-bMDP", 3);
    algorithm->getUpperBound()->setInitializer(sdm::initializer::registry::make("Mdp", algorithm->getWorld()));
    algorithm->initialize();
    algorithm->solve();
    resultat = algorithm->getLowerBound()->getValueAt(algorithm->getWorld()->getInitialState());
    BOOST_CHECK_EQUAL(round(resultat * 10000), 130155);
}