#define BOOST_TEST_MODULE ParserTest

#include <boost/test/unit_test.hpp>
#include <sdm/common.hpp>
#include <sdm/parser/parser.hpp>

BOOST_AUTO_TEST_CASE(ParserMDPTest)
{
    auto mdp_tiger = sdm::parser::parseMDP(sdm::tools::getWorldPath("tiger.mdp"));

    auto state_left = mdp_tiger->getStateSpace()->toDiscreteSpace()->getItem(0)->toState();
    auto open_left = mdp_tiger->getActionSpace()->toDiscreteSpace()->getItem(0)->toAction();
    BOOST_CHECK_EQUAL(mdp_tiger->getReward(state_left, open_left), -100);
    BOOST_CHECK_THROW(sdm::parser::parsePOMDP(sdm::tools::getWorldPath("tiger.mdp")), sdm::exception::Exception);
    BOOST_CHECK_THROW(sdm::parser::parseMPOMDP(sdm::tools::getWorldPath("tiger.mdp")), sdm::exception::Exception);
    BOOST_CHECK_THROW(sdm::parser::parsePOSG(sdm::tools::getWorldPath("tiger.mdp")), sdm::exception::Exception);
}

BOOST_AUTO_TEST_CASE(ParserPOMDPTest)
{
    auto pomdp_tiger = sdm::parser::parsePOMDP(sdm::tools::getWorldPath("tiger.pomdp"));

    auto state_left = pomdp_tiger->getStateSpace()->toDiscreteSpace()->getItem(0)->toState();
    auto open_left = pomdp_tiger->getActionSpace()->toDiscreteSpace()->getItem(0)->toAction();
    BOOST_CHECK_EQUAL(pomdp_tiger->getReward(state_left, open_left), -100);
    BOOST_CHECK_THROW(sdm::parser::parseMDP(sdm::tools::getWorldPath("tiger.pomdp")), sdm::exception::Exception);
    BOOST_CHECK_THROW(sdm::parser::parseMMDP(sdm::tools::getWorldPath("tiger.pomdp")), sdm::exception::Exception);
    BOOST_CHECK_THROW(sdm::parser::parsePOSG(sdm::tools::getWorldPath("tiger.pomdp")), sdm::exception::Exception);
}

BOOST_AUTO_TEST_CASE(ParserMPOMDPTest)
{
    auto dpomdp_tiger = sdm::parser::parseMPOMDP(sdm::tools::getWorldPath("tiger.dpomdp"));

    auto state_left = dpomdp_tiger->getStateSpace()->toDiscreteSpace()->getItem(0)->toState();
    auto open_left = dpomdp_tiger->getActionSpace()->toDiscreteSpace()->getItem(0)->toAction();
    BOOST_CHECK_EQUAL(dpomdp_tiger->getReward(state_left, open_left), -50);
    BOOST_CHECK_THROW(sdm::parser::parseMDP(sdm::tools::getWorldPath("tiger.dpomdp")), sdm::exception::Exception);
    BOOST_CHECK_THROW(sdm::parser::parsePOMDP(sdm::tools::getWorldPath("tiger.dpomdp")), sdm::exception::Exception);
}

BOOST_AUTO_TEST_CASE(ParserPOSGTest)
{
    auto posg_tiger = sdm::parser::parsePOSG(sdm::tools::getWorldPath("tiger.posg"));

    auto state_left = posg_tiger->getStateSpace()->toDiscreteSpace()->getItem(0)->toState();
    auto open_left = posg_tiger->getActionSpace()->toDiscreteSpace()->getItem(0)->toAction();
    BOOST_CHECK_EQUAL(posg_tiger->getReward(state_left, open_left, 0), -50);
    BOOST_CHECK_EQUAL(posg_tiger->getReward(state_left, open_left, 1), -50);
    BOOST_CHECK_THROW(sdm::parser::parseMDP(sdm::tools::getWorldPath("tiger.posg")), sdm::exception::Exception);
    BOOST_CHECK_THROW(sdm::parser::parseMMDP(sdm::tools::getWorldPath("tiger.posg")), sdm::exception::Exception);
    BOOST_CHECK_THROW(sdm::parser::parsePOMDP(sdm::tools::getWorldPath("tiger.posg")), sdm::exception::Exception);
    BOOST_CHECK_THROW(sdm::parser::parseMPOMDP(sdm::tools::getWorldPath("tiger.posg")), sdm::exception::Exception);
}