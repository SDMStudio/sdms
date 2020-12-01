#define BOOST_TEST_MODULE ParserTest

#include <boost/test/unit_test.hpp>
#include <sdm/common.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/worlds.hpp>

BOOST_AUTO_TEST_CASE(dpomdpParser)
{

    sdm::DecPOMDP dpomdp_world = sdm::parser::parse_file("../data/world/dpomdp/mabc.dpomdp");
    sdm::DecPOMDP dpomdp_2 = dpomdp_world;
    sdm::DecPOMDP dpomdp_3("../data/world/dpomdp/tiger.dpomdp");

    BOOST_CHECK_EQUAL(dpomdp_world.getNumAgents(), 2);
    BOOST_CHECK_EQUAL(dpomdp_world.getNumStates(), 4);
    BOOST_CHECK_EQUAL(dpomdp_world.getNumJActions(), 4);
    BOOST_CHECK_EQUAL(dpomdp_world.getNumJObservations(), 4);
}

BOOST_AUTO_TEST_CASE(zsposgParser)
{
    sdm::ZSPOSG zs_posg1 = sdm::parser::parse_file("../data/world/zsposg/fake_prisoners.zsposg");

    BOOST_CHECK_EQUAL(zs_posg1.getNumAgents(), 2);
    BOOST_CHECK_EQUAL(zs_posg1.getNumStates(), 1);
    BOOST_CHECK_EQUAL(zs_posg1.getNumJActions(), 4);
    BOOST_CHECK_EQUAL(zs_posg1.getNumJObservations(), 1);
}