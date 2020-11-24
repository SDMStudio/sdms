#define BOOST_TEST_MODULE ParserTest

#include <boost/test/unit_test.hpp>
#include <sdm/common.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/core/dpomdp.hpp>

BOOST_AUTO_TEST_CASE(dpomdpParser)
{

    sdm::dpomdp dpomdp_world = sdm::parser::parse_file("../data/world/dpomdp/mabc.dpomdp");

    BOOST_CHECK_EQUAL(dpomdp_world.getNumAgents(), 2);
    BOOST_CHECK_EQUAL(dpomdp_world.getNumStates(), 4);
    BOOST_CHECK_EQUAL(dpomdp_world.getNumActions(), 4);
    BOOST_CHECK_EQUAL(dpomdp_world.getNumObservations(), 4);
}
