#define BOOST_TEST_MODULE SpaceTest

#include <boost/test/unit_test.hpp>
#include <sdm/types.hpp>
#include <sdm/spaces.hpp>
#include <sdm/core/base_item.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>

using namespace sdm;

BOOST_AUTO_TEST_CASE(DiscreteSpaceTest)
{
    auto i1 = std::make_shared<NumberItem>(1), i2 = std::make_shared<NumberItem>(2), i3 = std::make_shared<NumberItem>(3);

    DiscreteSpace ds1({i1, i2, i3}), ds2({i1, i2, i3});

    BOOST_CHECK_EQUAL(ds1.getNumItems(), 3);
    BOOST_CHECK_EQUAL(ds1.getItem(0), i1);
    BOOST_CHECK_EQUAL(ds1.getItemIndex(i3), 2);
    BOOST_ASSERT(ds1 == ds2);
}

BOOST_AUTO_TEST_CASE(MultiDiscreteSpaceTest)
{
    auto up = std::make_shared<StringItem>("up"), down = std::make_shared<StringItem>("down"), fire = std::make_shared<StringItem>("fire");
    std::vector<std::vector<std::shared_ptr<Item>>> values = {{up, fire, fire, down}, {up, down, fire}};

    MultiDiscreteSpace mds1(values);
    auto ds1 = std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{up, down}), ds2 = std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>{up, down, fire, up});
    MultiDiscreteSpace mds2({ds1, ds2});

    BOOST_CHECK_EQUAL(mds1.getNumSpaces(), 2);
    BOOST_CHECK_EQUAL(mds1.getNumItems(), 12);
    BOOST_CHECK_EQUAL(mds1.getSpace(0)->toDiscreteSpace()->getNumItems(), 4);
    BOOST_CHECK_EQUAL(mds1.getSpace(1)->toDiscreteSpace()->getNumItems(), 3);

    BOOST_CHECK_EQUAL(mds2.getNumSpaces(), 2);
    BOOST_CHECK_EQUAL(mds2.getNumItems(), 8);
    BOOST_CHECK_EQUAL(mds2.getSpace(0)->toDiscreteSpace()->getItem(0), up);
}
