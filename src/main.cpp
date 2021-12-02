#define __main_program__
#include <iostream>
#include <boost/program_options.hpp>

#include <sdm/common.hpp>
#include <sdm/types.hpp>
#include <sdm/algorithms.hpp>
#include <sdm/worlds.hpp>

#include "programs/solve.cpp"

using namespace sdm;
using namespace std;
namespace po = boost::program_options;

void print_version()
{
  std::cout << std::endl
            << "SDMS Version :\t" << 1.0 << std::endl;
  std::cout << "CXX Version  :\t" << 17 << std::endl;
  std::cout << "CXX Compiler :\t"
            << "clang" << std::endl;
  std::cout << "Authors      :\t"
            << "David Albert" << std::endl
            << std::endl;
}

void print_help()
{
  std::cout << std::endl
            << "Usage : SDMStudio COMMAND" << std::endl
            << std::endl;
  std::cout << "The best solver for sequential decision making problems." << std::endl
            << std::endl;
  std::cout << "Commands:" << std::endl;
  std::cout << "  algorithms\t\tDisplay all available algorithms." << std::endl;
  std::cout << "  formalisms\t\tDisplay all available formalisms." << std::endl;
  std::cout << "  help\t\t\tShow this help message." << std::endl;
  std::cout << "  solve\t\t\tSolve a sequential decision making problem using specified algorithm." << std::endl;
  std::cout << "  test\t\t\tTest a policy." << std::endl;
  std::cout << "  version\t\tShow the version." << std::endl;
  std::cout << "  worlds\t\tDisplay all available worlds." << std::endl
            << std::endl;
  std::cout << "Run 'SDMStudio COMMAND --help' for more information on a command." << std::endl
            << std::endl;
}

void print_worlds()
{
  std::cout << "WORLDS\t" << std::endl
            << "---" << std::endl;
  std::string command = "ls -l " + config::PROBLEM_PATH + "*/*.* | cut -d/ -f8";
  int result = std::system(command.c_str());
}

int main_sdms(int argv, char **args)
{
  if (argv > 1)
  {
    string func(args[1]);
    // DO HELP
    if (func.compare("help") == 0 || func.compare("--help") == 0 || func.compare("-h") == 0)
    {
      print_help();
    }
    // DO VERSION
    else if (func.compare("version") == 0 || func.compare("--version") == 0 || func.compare("-v") == 0)
    {
      print_version();
    }
    // DO SOLVE
    else if (func.compare("solve") == 0)
    {
      solve(argv, args);
    }
    // DO TEST
    else if (func.compare("test") == 0)
    {
      std::cout << "Testing is not yet defined" << std::endl;
    }
    // LIST ALGORTIHMS
    else if (func.compare("algorithms") == 0)
    {
      std::cout << "ALGORITHMS\t" << std::endl
                << "---" << std::endl;
      for (auto algo : sdm::algo::available())
      {
        std::cout << algo << std::endl;
      }
    }
    // LIST WORLDS
    else if (func.compare("worlds") == 0)
    {
      print_worlds();
    }
    // LIST FORMALISMS
    else if (func.compare("formalisms") == 0)
    {
      std::cout << "FORMALISMS\t" << std::endl
                << "---" << std::endl;
      for (auto world : sdm::world::registry::available())
      {
        std::cout << world << std::endl;
      }
    }
    // DO OTHER
    else
    {
      cout << "Error: unrecognised command '" << func << "'" << endl;
      print_help();
      return ERROR_IN_COMMAND_LINE;
    }
  }
  else
  {
    cout << "Error: must specify a command" << endl;
    print_help();
    return ERROR_IN_COMMAND_LINE;
  }
  return SUCCESS;
}

int main(int argv, char **args)
{
  return main_sdms(argv, args);
}
