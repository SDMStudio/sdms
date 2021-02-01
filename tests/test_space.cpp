#define BOOST_TEST_MODULE SpaceTest

#include <boost/test/unit_test.hpp>
#include <sdm/types.hpp>
#include <sdm/spaces.hpp>

BOOST_AUTO_TEST_CASE(DiscreteSpaceTest)
{
    sdm::DiscreteSpace<sdm::number> ds1({0, 1, 2, 3, 4}), ds2({0, 1, 2, 3, 4});

    BOOST_CHECK_EQUAL(ds1.getNumItems(), 5);
    BOOST_CHECK_EQUAL(ds1.getItem(0), 0);
    BOOST_CHECK_EQUAL(ds1.getItemIndex(0), 0);
    BOOST_ASSERT(ds1 == ds2);

    sdm::DiscreteSpace<std::string> ds3({"up", "down", "left", "right"}), ds4({"up", "down", "right"}), ds5 = ds3;

    BOOST_CHECK_EQUAL(ds3.getNumItems(), 4);
    BOOST_CHECK_EQUAL(ds3.getItem(0), "up");
    BOOST_CHECK_EQUAL(ds3.getItemIndex("up"), 0);

    BOOST_ASSERT(ds3 != ds4);
    BOOST_ASSERT(ds3 == ds5);
}

BOOST_AUTO_TEST_CASE(MultiDiscreteSpaceTest)
{
    std::vector<std::vector<sdm::number>> values = {{2, 4, 5, 7}, {1, 2, 3}};
    sdm::MultiDiscreteSpace<sdm::number> mds1(values);
    
    sdm::DiscreteSpace<std::string> ds1({"up", "down"}), ds2({"up", "down", "left", "right"});
    sdm::MultiDiscreteSpace<std::string> mds2({ds1, ds2});

    BOOST_CHECK_EQUAL(mds1.getNumSpaces(), 2);
    BOOST_CHECK_EQUAL(mds1.getNumJointItems(), 12);
    BOOST_CHECK_EQUAL(mds1.getSpace(0)->getNumItems(), 4);
    BOOST_CHECK_EQUAL(mds1.getSpace(1)->getNumItems(), 3);

    BOOST_CHECK_EQUAL(mds2.getNumSpaces(), 2);
    BOOST_CHECK_EQUAL(mds2.getNumJointItems(), 8);
    BOOST_CHECK_EQUAL(mds2.getSpace(0)->getItem(0), "up");
}
