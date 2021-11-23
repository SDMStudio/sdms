#include <iostream>
#include <sdm/utils/value_function/update_operator/registry.hpp>

int main(int argc, char **argv)
{
    std::cout << sdm::update::registry::available() << std::endl;

    // auto update_op = sdm::update::registry::registry::make("TabularUpdate", nullptr);

} // END main