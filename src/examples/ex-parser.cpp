#include <iostream>
#include <sdm/config.hpp>
#include <sdm/parser/parser.hpp>

int main(int argc, char **argv)
{
    std::string filename = (argc > 1) ? argv[1] : sdm::config::PROBLEM_PATH + "dpomdp/mabc.dpomdp";
    auto problem = sdm::parser::parse_file(filename);
    std::cout << *problem << std::endl;
    return 0;
}