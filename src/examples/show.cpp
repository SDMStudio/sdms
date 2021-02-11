#include <iostream>
#include <boost/program_options.hpp>

namespace
{
    const size_t SUCCESS = 0;
    const size_t ERROR_IN_COMMAND_LINE = 1;
    const size_t ERROR_UNHANDLED_EXCEPTION = 2;
} // namespace

int main(int argv, char **args)
{

    try
    {
        namespace po = boost::program_options;
        po::options_description desc("Allowed options");

        std::cout << "#> Show program !!!" << std::endl;
    }
    catch (std::exception &e)
    {
        std::cerr << "Unhandled Exception reached the top of main: " << e.what() << ", application will now exit" << std::endl;
        return ERROR_UNHANDLED_EXCEPTION;
    }

    return SUCCESS;
}
