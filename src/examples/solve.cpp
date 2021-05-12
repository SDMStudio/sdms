#include <iostream>
#include <boost/program_options.hpp>

#include <sdm/types.hpp>
#include <sdm/common.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/algorithms.hpp>

using namespace sdm;
using namespace std;
namespace po = boost::program_options;

int solve(int argv, char **args)
{
    try
    {
        std::string problem, algorithm, formalism, name, upper_bound, lower_bound, ub_init, lb_init;
        int trials, truncation;
        number horizon;
        double error, discount;

        po::options_description options("Options");
        options.add_options()
        ("help", "produce help message")
        ("test", "test the policy found");

        po::options_description config("Configuration");
        config.add_options()
        ("algorithm,a", po::value<string>(&algorithm)->default_value("hsvi"), "the algorithm to use")
        ("problem,p", po::value<string>(&problem)->default_value("tiger"), "the problem to be solved")
        ("formalism,f", po::value<string>(&formalism)->default_value("decpomdp"), "the formalism to use")
        ("error,e", po::value<double>(&error)->default_value(0.001), "the error")
        ("discount,d", po::value<double>(&discount)->default_value(0.9), "the discount factor")
        ("horizon,h", po::value<number>(&horizon)->default_value(0), "the planning horizon")
        ("trials,t", po::value<int>(&trials)->default_value(100000), "the maximum number of trials")
        ("truncation", po::value<int>(&truncation)->default_value(-1), "the truncation parameter (for history)")
        ("name,n", po::value<std::string>(&name)->default_value(""), "the name of the experiment");

        po::options_description algo_config("Algorithms configuration");
        algo_config.add_options()
        ("lower_bound", po::value<string>(&lower_bound)->default_value("tabular"), "the lower bound representation (HSVI, ValurIteration)")
        ("upper_bound", po::value<string>(&upper_bound)->default_value("tabular"), "the upper bound representation (HSVI)")
        ("lb_init", po::value<string>(&lb_init)->default_value("MinInitializer"), "the lower bound initialization method (HSVI, ValurIteration)")
        ("ub_init", po::value<string>(&ub_init)->default_value("MaxInitializer"), "the upper bound initialization method (HSVI)");

        po::options_description visible("\nUsage:\tsdms-solve [CONFIGS]\n\tSDMStudio solve [CONFIGS]\n\nSolve a problem with specified algorithms and configurations.");
        visible.add(options).add(config).add(algo_config);

        po::options_description config_file_options;
        config_file_options.add(config);

        po::variables_map vm;
        try
        {
            po::store(po::command_line_parser(argv, args).options(visible).run(), vm);
            po::notify(vm);
            if (vm.count("help"))
            {
                std::cout << visible << std::endl;
                return sdm::SUCCESS;
            }
        }
        catch (po::error &e)
        {
            std::cerr << "ERROR: " << e.what() << std::endl;
            std::cerr << visible << std::endl;
            return sdm::ERROR_IN_COMMAND_LINE;
        }

        std::vector<std::string> av_algos = sdm::algo::available();
        if (std::find(av_algos.begin(), av_algos.end(), algorithm) != av_algos.end())
        {
            auto algo = sdm::algo::make(algorithm, problem, formalism, upper_bound, lower_bound, ub_init, lb_init, discount, error, horizon, trials, truncation, name);
            algo->do_initialize();
            algo->do_solve();

            if (vm.count("test"))
            {
                algo->do_test();
            }
        }
        else
        {
            std::cout << "Error: " << algorithm << " is not a valid algorithm." << std::endl
                      << std::endl;
            std::cout << "#> Available algorithms are : " << std::endl;
            std::cout << "ALGORITHM\t" << std::endl;
            for (auto algo : sdm::algo::available())
            {
                std::cout << algo << std::endl;
            }
        }
    }
    catch (std::exception &e)
    {
        std::cerr << "Unhandled Exception reached the top of main: " << e.what() << ", application will now exit" << std::endl;
        return sdm::ERROR_UNHANDLED_EXCEPTION;
    }

    return sdm::SUCCESS;
}

#ifndef __main_program__
#define __main_program__
int main(int argv, char **args)
{
    return solve(argv, args);
}
#endif