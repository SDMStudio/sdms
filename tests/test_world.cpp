#define BOOST_TEST_MODULE WorldTest

#include <boost/test/unit_test.hpp>
#include <sdm/common.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/worlds.hpp>

BOOST_AUTO_TEST_CASE(dpomdpWorldTest)
{
    auto dpomdp_world = sdm::parser::parse_file(sdm::tools::getWorldPath("mabc.dpomdp"));

    BOOST_CHECK_EQUAL(dpomdp_world->getNumAgents(), 2);
    BOOST_CHECK_EQUAL(dpomdp_world->getStateSpace()->toDiscreteSpace()->getNumItems(), 4);
    BOOST_CHECK_EQUAL(dpomdp_world->getActionSpace()->toDiscreteSpace()->getNumItems(), 4);
    BOOST_CHECK_EQUAL(dpomdp_world->getObservationSpace()->toDiscreteSpace()->getNumItems(), 4);
}