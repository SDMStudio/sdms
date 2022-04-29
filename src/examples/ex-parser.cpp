#include <iostream>
#include <sdm/config.hpp>
#include <sdm/parser/parser.hpp>

int main(int argc, char **argv)
{
    // std::string filename = (argc > 1) ? argv[1] : sdm::config::PROBLEM_PATH + "dpomdp/mabc.dpomdp";
    // auto problem = sdm::parser::parse_file(filename);
    // std::cout << *problem << std::endl;

    // std::string filename2 = sdm::config::PROBLEM_PATH + "mdp/tiger.mdp";
    // auto problem2 = sdm::parser::parseMDP(filename2);
    // std::cout << *problem2 << std::endl;

    std::string filename1 = sdm::config::PROBLEM_PATH + "mmdp/tiger.mmdp";
    auto problem1 = sdm::parser::parseMMDP(filename1);
    std::cout << *problem1 << std::endl;

    std::string filename3 = sdm::config::PROBLEM_PATH + "pomdp/tiger.pomdp";
    auto problem3 = sdm::parser::parsePOMDP(filename3);
    std::cout << *problem3 << std::endl;

    return 0;
}