#define BOOST_TEST_MODULE SpaceTest

#include <boost/test/unit_test.hpp>
#include <sdm/core/discrete_space.hpp>
#include <sdm/core/multi_discrete_space.hpp>

BOOST_AUTO_TEST_CASE(DiscreteSpaceTest)
{
    sdm::DiscreteSpace ds1(8), ds2({"up", "down", "left", "right"});

    BOOST_CHECK_EQUAL(ds1.getLength(), 8);
    BOOST_CHECK_EQUAL(ds1.getElementName(7), "7");

    BOOST_CHECK_EQUAL(ds2.getLength(), 4);
    BOOST_CHECK_EQUAL(ds2.getElementName(0), "up");
    BOOST_CHECK_EQUAL(ds2.getElementIndex("down"), 1);

    sdm::DiscreteSpace ds3(8), ds4({"up", "down", "left", "right"});

    BOOST_ASSERT(ds1 == ds1);
    BOOST_ASSERT(ds1 == ds3);
    BOOST_ASSERT(ds2 == ds2);
    BOOST_ASSERT(ds2 == ds4);
    BOOST_ASSERT(ds1 != ds2);

    ds1 = ds2;
    BOOST_ASSERT(ds1 == ds2);
}

BOOST_AUTO_TEST_CASE(MultiDiscreteSpaceTest)
{   
    sdm::DiscreteSpace ds1(8), ds2({"up", "down", "left", "right"});
    std::vector<int> n_els = {2,3,4,2};
    sdm::MultiDiscreteSpace mds1(n_els), mds2({{"up", "down", "left", "right"}, {"up", "down"}}), mds3(std::vector<int>{2,2}), mds4({ds1, ds2});

    BOOST_CHECK_EQUAL(mds1.getNumSpaces(), 4);
    BOOST_CHECK_EQUAL(mds1.getNumJElements(), 48);
    BOOST_CHECK_EQUAL(mds1.getSpace(1).getNumElements(), 3);

    BOOST_CHECK_EQUAL(mds2.getNumSpaces(), 2);
    BOOST_CHECK_EQUAL(mds2.getNumJElements(), 8);
    BOOST_CHECK_EQUAL(mds2.getSpace(0).getElementName(1), "down");
    BOOST_CHECK_EQUAL(mds2.getElementName(1,1), "down");
    BOOST_CHECK_EQUAL(mds2.getElementName(0,2), "left");
    BOOST_CHECK_EQUAL(mds2.getElementIndex(0, "left"), 2);


    BOOST_ASSERT(mds1 == mds1);
    BOOST_ASSERT(mds2 == mds2);
    BOOST_ASSERT(mds1 != mds2);

    mds1 = mds2;
    BOOST_ASSERT(mds1 == mds2);
}
