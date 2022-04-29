#include <iostream>
#include <sdm/types.hpp>
#include <sdm/core/item.hpp>
#include <sdm/core/base_item.hpp>
#include <sdm/core/space/iterator/super_iterator.hpp>
#include <sdm/core/space/iterator/combination_iterator.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    // Instanciate 3 different items
    std::shared_ptr<Item> item1 = std::make_shared<BaseItem<std::string>>("item1");
    std::shared_ptr<Item> item2 = std::make_shared<BaseItem<int>>(2);
    std::shared_ptr<Item> item3 = std::make_shared<BaseItem<double>>(0.96);

    // Add the 3 items in a list
    std::vector<std::shared_ptr<Item>> list1{item1, item2, item3};

    // Instanciate a super iterator that will iterate over items in the list
    std::shared_ptr<ItemIterator> begin_iterator = std::make_shared<iterator::SuperIterator<std::shared_ptr<Item>, decltype(list1.begin())>>(list1.begin());
    std::shared_ptr<ItemIterator> end_iterator = std::make_shared<iterator::SuperIterator<std::shared_ptr<Item>, decltype(list1.end())>>(list1.end());

    std::cout << "------ SuperIterator -------" << std::endl;

    // Iterate over items and display them
    for (auto item_iterator = begin_iterator; !item_iterator->equal(end_iterator); item_iterator->next())
    {
        std::cout << item_iterator->getCurrent()->str() << std::endl;
    }

    // Add the 3 items in a list
    std::vector<std::shared_ptr<Item>> list2{item1, item2, item3};

    // Reinit begin and end iterator over list 1
    begin_iterator = std::make_shared<iterator::SuperIterator<std::shared_ptr<Item>, decltype(list1.begin())>>(list1.begin());
    end_iterator = std::make_shared<iterator::SuperIterator<std::shared_ptr<Item>, decltype(list1.end())>>(list1.end());

    // Instanciate a super iterator that will iterate over the second list
    std::shared_ptr<ItemIterator> begin_iterator2 = std::make_shared<iterator::SuperIterator<std::shared_ptr<Item>, decltype(list2.begin())>>(list2.begin());
    std::shared_ptr<ItemIterator> end_iterator2 = std::make_shared<iterator::SuperIterator<std::shared_ptr<Item>, decltype(list2.end())>>(list2.end());

    // Instanciate a combination iterator that will iterate over all combination of the two lists
    std::vector<std::shared_ptr<ItemIterator>> begins{begin_iterator, begin_iterator}, ends{end_iterator, end_iterator};

    std::shared_ptr<ItemIterator> begin_combination_iterator = std::make_shared<sdm::iterator::CombinationIterator<std::shared_ptr<Item>>>(begins, ends);
    std::shared_ptr<ItemIterator> end_combination_iterator = std::make_shared<sdm::iterator::CombinationIterator<std::shared_ptr<Item>>>();

    std::cout << "------ CombinationIterator -------" << std::endl;
    // Iterate over items and display them
    for (auto item_iterator = begin_combination_iterator; !item_iterator->equal(end_combination_iterator); item_iterator->next())
    {
        std::cout << item_iterator->getCurrent()->str() << std::endl;
    }
}
