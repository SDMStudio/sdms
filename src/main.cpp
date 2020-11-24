#include <time.h>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <boost/program_options.hpp>

#include <sdm/common.hpp>

///////////////////////////////////////////////////////////////////////////////
//  Main program
///////////////////////////////////////////////////////////////////////////////
namespace
{
  const size_t SUCCESS = 0;
  const size_t ERROR_IN_COMMAND_LINE = 1;
  const size_t ERROR_UNHANDLED_EXCEPTION = 2;
} // namespace

int main(int argv, char** args){

  std::cout << "#> Launching SDM'Studio !!!" << std::endl;
  sdm::common::logo();

  return SUCCESS;
}
