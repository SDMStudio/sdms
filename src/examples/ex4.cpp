/**
 * @file ex4.cpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief File that give some usage exemples of using spaces
 * @version 1.0
 * @date 01/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <iostream>
#include <cassert>
#include <sdm/worlds.hpp>
#include <sdm/spaces.hpp>
#include <sdm/core/action/det_decision_rule.hpp>
#include <sdm/exception.hpp>

using namespace sdm;

int main(int argc, char **argv)
{
    // DiscreteSpace contain a list of possible items any type (string in the following exemple)
    DiscreteSpace<std::string> hello_dsp({"Hello", "Hi", "Good Morning"});
    DiscreteSpace<std::string> bye_dsp({"Bye", "See you"});

    auto all_dsp = hello_dsp.getAll();            // return {"Hello", "Hi", "Good Morning"}
    std::cout << hello_dsp.sample() << std::endl; // sample a random item and display it (ex: "Hi")

    // MultiDiscreteSpace are a set of DiscreteSpace
    // -> they can be viewed both as a DiscreteSpace of Joint<Item> or as a MultiSpace of DiscreteSpace
    MultiDiscreteSpace<std::string> str_mdsp({hello_dsp, bye_dsp});

    auto all_mdsp = str_mdsp.getAll();           // return {{"Hello", "Bye"}, {"Hello", "See you"}, {"Hi", "Bye"}, {"Hi", "See you"}, {"Good Morning", "Bye"},{"Good Morning", "See you"},}
    std::cout << str_mdsp.sample() << std::endl; // sample a random item and display it (ex : {"Good Morning", "Bye"})

    // FunctionSpace are DiscreteSpace of functions
    // -> FunctionSpace are initialize given an input space and an output space
    FunctionSpace<DeterministicDecisionRule<std::string, std::string>> str_fsp(hello_dsp, bye_dsp);

    auto all_fsp = str_fsp.getAll();            // return the list of all possible function
    std::cout << str_fsp.sample() << std::endl; // sample a random function and display it

    return 0;
}
