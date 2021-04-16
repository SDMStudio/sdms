#include <iostream>
#include <boost/program_options.hpp>

#include <sdm/types.hpp>
#include <sdm/common.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/algorithms.hpp>

using namespace sdm;
using namespace std;
namespace po = boost::program_options;

int learn(int argv, char **args)
{
    try
    {
        std::string problem_path, algorithm_name, formalism, name, qvalue_type, q_init_method;
        unsigned long nb_timesteps;
        number horizon;
        double lr, discount_factor;

        po::options_description options("Options");
        options.add_options()
        ("help", "produce help message")
        ("test", "test the policy found");

        po::options_description config("Configuration");
        config.add_options()
        ("algorithm,a", po::value<string>(&algorithm_name)->default_value("qlearning"), "the learning algorithm to use")
        ("problem,p", po::value<string>(&problem_path)->default_value("tiger"), "the problem to be solved")
        ("formalism,f", po::value<string>(&formalism)->default_value("decpomdp"), "the formalism to use")
        ("lr,l", po::value<double>(&lr)->default_value(0.01), "the learning rate")
        ("discount,d", po::value<double>(&discount_factor)->default_value(0.9), "the discount factor")
        ("horizon,h", po::value<number>(&horizon)->default_value(0), "the planning horizon. If 0 then infinite horizon.")
        ("nb_timesteps,t", po::value<unsigned long>(&nb_timesteps)->default_value(100000), "the maximum number of timesteps")
        ("name,n", po::value<std::string>(&name)->default_value(""), "the name of the experiment");

        po::options_description algo_config("Algorithms configuration");
        algo_config.add_options()
        ("qvalue,q", po::value<string>(&qvalue_type)->default_value("tabular"), "the representation of the Q-Value")
        ("init,i", po::value<string>(&q_init_method)->default_value("ZeroInitializer"), "the Q-Value initialization method");

        po::options_description visible(
            "\nUsage:\tsdms-solve [CONFIGS]\n\tSDMStudio solve [CONFIGS]\n\nSolve a problem with specified algorithms and configurations."
        );
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
        if (std::find(av_algos.begin(), av_algos.end(), algorithm_name) != av_algos.end())
        {
            auto a = sdm::algo::make(
                algorithm_name, problem_path, formalism, qvalue_type, q_init_method, horizon, discount_factor, lr,  1, nb_timesteps, name
            );
            a->do_initialize();
            a->do_solve();

            if (vm.count("test"))
            {
                a->do_test();
            }
        }
        else
        {
            std::cout << "Error: " << algorithm_name << " is not a valid algorithm." << std::endl << std::endl;
            std::cout << "#> Available algorithms are : " << std::endl;
            std::cout << "ALGORITHM\t" << std::endl;
            for (auto a : sdm::algo::available())
            {
                std::cout << a << std::endl;
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
    return learn(argv, args);
}
#endif