#include <iostream>
#include <sdm/core/joint.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>

using namespace sdm;

int main(int, char **)
{
    std::cout << "----- Usage : DiscreteSpace && MultiDiscreteSpace ( sdm/core/discrete_space.hpp ) ---------" << std::endl
              << std::endl;

    // List of list of possible items
    std::vector<std::vector<number>> v = {{0, 1, 2}, {0, 1, 2}};

    // Instanciate a multi discrete space from a list of possible items
    MultiDiscreteSpace<number> multi_space(v), copy;

    copy = multi_space;

    multi_space.setSpaces(std::vector<std::vector<number>>{{6, 1, 2}, {0, 3, 4}});

    // Accessors
    std::cout << "#> Num items = " << multi_space.getNumItems() << std::endl;
    std::cout << "#> MultiSpace \n"
              << multi_space << std::endl;
    std::cout << "#> Copy \n"
              << copy << std::endl;
    std::cout << "#> Are Equal ? " << (copy == multi_space) << std::endl;
    std::cout << "#> List all items = " << multi_space.getAll() << std::endl;
    std::cout << "#> Sample = " << multi_space.sample() << std::endl;

    Joint<number> joint_item = {6, 0};
    std::cout << "#> Get index of joint item \"" << joint_item << "\" --> " << multi_space.joint2single(joint_item) << std::endl;
    std::cout << "#> Get joint index = " << multi_space.single2joint(multi_space.joint2single(joint_item)) << std::endl;

    std::cout << "------------------------" << std::endl;
    
    // Another constructor
    MultiDiscreteSpace<number> multi_space_2({4, 2, 6});
    std::cout << "#> Num items = " << multi_space_2.getNumItems() << std::endl;
    std::cout << "#> MultiSpace \n"
              << multi_space_2 << std::endl;
    std::cout << "#> Copy \n"
              << copy << std::endl;
    std::cout << "#> Are Equal ? " << (copy == multi_space_2) << std::endl;
    std::cout << "#> List all items = " << multi_space_2.getAll() << std::endl;
    std::cout << "#> Sample = " << multi_space_2.sample() << std::endl;

    return 0;
}
