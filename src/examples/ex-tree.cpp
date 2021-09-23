
#include <iostream>
#include <sdm/types.hpp>
#include <sdm/config.hpp>
#include <sdm/utils/struct/tree.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    // TEST GRAPH
    std::cout << "\n--------- Usage : class Tree ( sdm/utils/struct/tree.hpp ) ---------\n\n";

    std::shared_ptr<Tree<int>> tree = std::make_shared<Tree<int>>(4), child;

    tree->addChildren({3, 4, 5});
    tree->getChild(3)->addChildren({9, 8, 7, 6});
    tree->getChild(5)->addChildren({1, 3});

    std::cout << config::LOG_SDMS << "root.Depth=" << tree->getDepth() << std::endl; // OUTPUT 0
    std::cout << config::LOG_SDMS << "root.NumChildren=" << tree->getNumChildren() << std::endl; // OUTPUT : 3

    child = tree->getChild(3);
    std::cout << config::LOG_SDMS << "GetChild(3).Depth=" << child->getDepth() << std::endl; // OUTPUT : 1
    std::cout << config::LOG_SDMS << "GetChild(3).NumChildren=" << child->getNumChildren() << std::endl; // OUTPUT : 4
}